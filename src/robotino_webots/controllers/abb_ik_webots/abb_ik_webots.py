#!/usr/bin/env python3
"""
abb_fake_driver.py

Webots "mock driver" that exposes joint-level control to ROS 2.

- Discovers motors using the Webots-exported URDF + ikpy Chain
- Subscribes to /arm_joint_target (sensor_msgs/msg/JointState)  [kept for quick tests]
- Publishes /joint_states (sensor_msgs/msg/JointState)          [added]
- Provides /follow_joint_trajectory action (control_msgs/action/FollowJointTrajectory) [added]
- Applies positions directly to Webots motors (motor.setPosition)
"""

import sys
import tempfile
import time
from dataclasses import dataclass
from typing import List, Optional

from controller import Supervisor

print("### RUNNING CONTROLLER FROM:", __file__)

# --- ikpy import (only used to discover links/motor names from URDF) ---
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. Install with: pip install ikpy')

if ikpy.__version__[0] < '3':
    sys.exit('ikpy version too old. Need >= 3.0')

# --- ROS2 ---
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


@dataclass
class ActiveTrajectory:
    joint_names: List[str]
    points_positions: List[List[float]]
    points_times: List[Duration]  # times_from_start
    start_time: Time
    goal_handle: any


class FakeDriverNode(Node):
    def __init__(self, joint_names: List[str]):
        super().__init__('abb_fake_driver')

        self.joint_names = list(joint_names)
        self.joint_target: Optional[List[float]] = None
        self.active_traj: Optional[ActiveTrajectory] = None

        # Reentrant group so action + subscription can run concurrently
        self.cb_group = ReentrantCallbackGroup()

        # Subscriber (kept)
        self.create_subscription(
            JointState,
            '/arm_joint_target',
            self.cb_joint_target,
            10,
            callback_group=self.cb_group
        )

        # Publisher (kept QoS that you confirmed works)
        qos_js = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.js_pub = self.create_publisher(JointState, '/joint_states', qos_js)

        # Action server
        self._as = ActionServer(
            self,
            FollowJointTrajectory,
            '/follow_joint_trajectory',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,
        )

    def cb_joint_target(self, msg: JointState):
        if msg.position:
            self.joint_target = list(msg.position)

    def goal_cb(self, goal_request: FollowJointTrajectory.Goal):
        traj = goal_request.trajectory

        if not traj.joint_names:
            self.get_logger().warn("Reject trajectory: empty joint_names")
            return GoalResponse.REJECT

        if list(traj.joint_names) != self.joint_names:
            self.get_logger().warn(
                "Reject trajectory: joint_names mismatch.\n"
                f"  expected: {self.joint_names}\n"
                f"  got:      {list(traj.joint_names)}"
            )
            return GoalResponse.REJECT

        if not traj.points:
            self.get_logger().warn("Reject trajectory: no points")
            return GoalResponse.REJECT

        prev_t_ns = -1
        for i, p in enumerate(traj.points):
            if not p.positions:
                self.get_logger().warn(f"Reject trajectory: point {i} has no positions")
                return GoalResponse.REJECT
            if len(p.positions) != len(self.joint_names):
                self.get_logger().warn(
                    f"Reject trajectory: point {i} positions len {len(p.positions)} "
                    f"!= {len(self.joint_names)}"
                )
                return GoalResponse.REJECT
            t_ns = Duration.from_msg(p.time_from_start).nanoseconds
            if t_ns <= prev_t_ns:
                self.get_logger().warn("Reject trajectory: times_from_start not strictly increasing")
                return GoalResponse.REJECT
            prev_t_ns = t_ns

        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel request received.")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory

        points_positions = [list(p.positions) for p in traj.points]
        points_times = [Duration.from_msg(p.time_from_start) for p in traj.points]

        self.active_traj = ActiveTrajectory(
            joint_names=list(traj.joint_names),
            points_positions=points_positions,
            points_times=points_times,
            start_time=self.get_clock().now(),
            goal_handle=goal_handle
        )

        self.get_logger().info(
            f"Executing trajectory with {len(points_positions)} points "
            f"(T_end={points_times[-1].nanoseconds/1e9:.3f}s)"
        )

        # Wait until main loop finishes it (executor threads keep callbacks alive)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Trajectory canceled.")
                self.active_traj = None
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            if self.active_traj is None:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            time.sleep(0.02)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        return result


def lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u


def interpolate_trajectory(traj: ActiveTrajectory, now: Time) -> Optional[List[float]]:
    """
    Piecewise-linear interpolation in joint space.
    Returns desired joint positions, or None if finished.
    Uses nanoseconds math (Jazzy Duration subtraction is not supported).
    """
    t = now - traj.start_time
    t_ns = t.nanoseconds

    if t_ns < 0:
        return traj.points_positions[0]

    # 1-point trajectory
    if len(traj.points_times) == 1:
        if t_ns >= traj.points_times[0].nanoseconds:
            return None
        return traj.points_positions[0]

    times = traj.points_times
    t_end_ns = times[-1].nanoseconds
    if t_ns >= t_end_ns:
        return None

    i = 0
    while i + 1 < len(times) and times[i + 1].nanoseconds <= t_ns:
        i += 1

    if i + 1 >= len(times):
        return None

    t0_ns = times[i].nanoseconds
    t1_ns = times[i + 1].nanoseconds
    dt_ns = t1_ns - t0_ns
    if dt_ns <= 0:
        return traj.points_positions[i + 1]

    u = (t_ns - t0_ns) / float(dt_ns)
    u = max(0.0, min(1.0, u))

    q0 = traj.points_positions[i]
    q1 = traj.points_positions[i + 1]
    return [lerp(q0[j], q1[j], u) for j in range(len(q0))]


def main():
    supervisor = Supervisor()
    time_step = int(4 * supervisor.getBasicTimeStep())

    rclpy.init(args=None)

    with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as f:
        urdf_path = f.name
        f.write(supervisor.getUrdf().encode('utf-8'))

    arm_chain = Chain.from_urdf_file(urdf_path)

    motors = []
    joint_names = []
    position_sensors = []

    for link in arm_chain.links:
        if 'motor' in link.name:
            motor = supervisor.getDevice(link.name)
            motor.setVelocity(1.0)
            ps = motor.getPositionSensor()
            ps.enable(time_step)

            motors.append(motor)
            joint_names.append(link.name)
            position_sensors.append(ps)

    rosnode = FakeDriverNode(joint_names)

    # ✅ This is the critical change: executor handles action callbacks without blocking Webots loop
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(rosnode)

    print('[abb_fake_driver] Ready.')
    print('[abb_fake_driver] Motors discovered (order matters):')
    for i, n in enumerate(joint_names):
        print(f'  {i}: {n}')
    print('[abb_fake_driver] motors len =', len(motors))

    while supervisor.step(time_step) != -1:
        # Process ROS callbacks without blocking the simulation loop
        executor.spin_once(timeout_sec=0.0)

        # Publish joint_states
        js = JointState()
        js.header.stamp = rosnode.get_clock().now().to_msg()
        js.name = list(joint_names)
        js.position = [ps.getValue() for ps in position_sensors]
        rosnode.js_pub.publish(js)

        # Priority 1: trajectory
        if rosnode.active_traj is not None:
            desired = interpolate_trajectory(rosnode.active_traj, rosnode.get_clock().now())
            if desired is None:
                gh = rosnode.active_traj.goal_handle
                gh.succeed()
                rosnode.get_logger().info("Trajectory succeeded.")
                rosnode.active_traj = None
            else:
                n = min(len(motors), len(desired))
                for i in range(n):
                    motors[i].setPosition(desired[i])
            continue

        # Priority 2: direct topic control (kept)
        if rosnode.joint_target is None:
            continue

        q = rosnode.joint_target
        n = min(len(motors), len(q))
        for i in range(n):
            motors[i].setPosition(q[i])


if __name__ == '__main__':
    main()
