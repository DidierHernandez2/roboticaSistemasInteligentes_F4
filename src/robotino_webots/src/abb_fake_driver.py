#!/usr/bin/env python3
"""
abb_fake_driver.py

A Webots "mock driver" that exposes joint-level control to ROS 2.

- Discovers motors using the Webots-exported URDF + ikpy Chain (same trick as abb_webots_controller.py)
- Subscribes to /arm_joint_target (sensor_msgs/JointState)
- Applies positions directly to Webots motors (motor.setPosition)

Future:
- publish /joint_states
- replace with ros2_control + joint_trajectory_controller for MoveIt
"""

import sys
import tempfile

from controller import Supervisor

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
from sensor_msgs.msg import JointState


class FakeDriverNode(Node):
    def __init__(self):
        super().__init__('abb_fake_driver')
        self.joint_target = None  # list[float]
        self.create_subscription(JointState, '/arm_joint_target', self.cb_joint_target, 10)

    def cb_joint_target(self, msg: JointState):
        # Minimal interface: only positions, order must match motors[]
        if msg.position:
            self.joint_target = list(msg.position)


def main():
    supervisor = Supervisor()
    time_step = int(4 * supervisor.getBasicTimeStep())

    # ROS init (inside Webots controller process)
    rclpy.init(args=None)
    rosnode = FakeDriverNode()

    # Export URDF from Webots and build Chain (for motor discovery/order)
    with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as f:
        urdf_path = f.name
        f.write(supervisor.getUrdf().encode('utf-8'))

    # IMPORTANT:
    # We don't care about IK here; we use the Chain to iterate links and find motor link names.
    # Keep this mask aligned with your arm chain if needed; it mainly affects link enumeration.
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

    print('[abb_fake_driver] Ready.')
    print('[abb_fake_driver] Motors discovered (order matters):')
    for i, n in enumerate(joint_names):
        print(f'  {i}: {n}')
    print('[abb_fake_driver] Command with:')
    print('  ros2 topic pub -1 /arm_joint_target sensor_msgs/msg/JointState "{position: [..]}"')

    while supervisor.step(time_step) != -1:
        rclpy.spin_once(rosnode, timeout_sec=0.0)

        if rosnode.joint_target is None:
            continue

        q = rosnode.joint_target
        n = min(len(motors), len(q))

        for i in range(n):
            motors[i].setPosition(q[i])


if __name__ == '__main__':
    main()
