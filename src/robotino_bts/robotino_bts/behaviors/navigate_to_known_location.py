#!/usr/bin/env python3
import math
from tf_transformations import quaternion_from_euler
import py_trees
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool


class NavToKnownLocation(py_trees.behaviour.Behaviour):
    """
    Nav2 to named location with:
    - Pre-rotate hack (cancel on first / plan / publish yaw from lookahead / wait /align / resend)
    - Always final yaw correction after arrival using the **original goal yaw**
    """

    def __init__(self, name, node, location_name, timeout_sec=60.0,
                 plan_topic="/transformed_global_plan",
                 yaw_topic="/target_yaw",
                 lookahead_idx=3,
                 align_timeout_sec=18.0):  # increased default
        super().__init__(name)
        self.node = node
        self.location_name = location_name
        self.timeout_sec = float(timeout_sec)
        #self.plan_topic = plan_topic   # MPPI IMPORTANT DO NOT REMOVE
        self.plan_topic = "/plan"
        self.yaw_topic = yaw_topic
        self.lookahead_idx = int(lookahead_idx)
        self.align_timeout_sec = float(align_timeout_sec)

        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._yaw_pub = node.create_publisher(Float32, self.yaw_topic, 10)
        self._plan_sub = node.create_subscription(Path, self.plan_topic, self._on_plan, 10)
        self._yaw_aligned_sub = node.create_subscription(
            Bool, "/yaw_aligned", self._on_yaw_aligned, 10
        )

        # Runtime state
        self._goal_msg = None
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._start_time = None
        self._latest_plan = None
        self._saw_plan = False
        self._canceled_once = False
        self._resent = False
        self._yaw_aligned = False
        self._pre_yaw_time = None
        self._final_yaw_time = None
        self._final_yaw_published = False
        self._target_yaw = 0.0  # will store the original yaw
        self.log_once = True
    def _on_yaw_aligned(self, msg: Bool):
        self._yaw_aligned = bool(msg.data)

    def _on_plan(self, msg: Path):
        if msg is None or len(msg.poses) < 2:
            return
        self._latest_plan = msg
        self._saw_plan = True

    def _yaw_from_plan(self, plan: Path, lookahead_idx: int):
        i = min(max(1, lookahead_idx), len(plan.poses) - 1)
        p0 = plan.poses[0].pose.position
        p1 = plan.poses[i].pose.position
        return math.atan2(p1.y - p0.y, p1.x - p0.x)

    def _publish_target_yaw(self, yaw_rad: float):
        msg = Float32(data=float(yaw_rad))
        self._yaw_pub.publish(msg)
        return self.node.get_clock().now()

    def initialise(self):
        # Reset everything
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._start_time = self.node.get_clock().now()
        self._latest_plan = None
        self._saw_plan = False
        self._canceled_once = False
        self._resent = False
        self._yaw_aligned = False
        self._pre_yaw_time = None
        self._final_yaw_time = None
        self._final_yaw_published = False
        self._target_yaw = 0.0
        self.log_once = True
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Nav2 action server not available")
            return

        loc = getattr(self.node, "known_locations", {}).get(self.location_name)
        if not loc:
            self.node.get_logger().error(f"Unknown location: {self.location_name}")
            return

        x, y = float(loc["x"]), float(loc["y"])
        self._target_yaw = float(loc.get("yaw", 0.0))  # ← this is what we always use for final too
        frame = loc.get("frame", "map")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, self._target_yaw)
        goal.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self._goal_msg = goal
        self._goal_future = self._client.send_goal_async(goal)

        self.node.get_logger().info(
            f"NavTo '{self.location_name}' → ({x:.2f}, {y:.2f}, yaw={self._target_yaw:.2f})"
        )

    def update(self):
        now = self.node.get_clock().now()
        elapsed = (now - self._start_time).nanoseconds / 1e9

        if elapsed > self.timeout_sec:
            self.node.get_logger().warn("Overall timeout")
            return py_trees.common.Status.FAILURE

        if self._goal_future is None:
            return py_trees.common.Status.FAILURE

        # 1. Wait for goal acceptance
        if self._goal_handle is None:
            if not self._goal_future.done():
                return py_trees.common.Status.RUNNING
            self._goal_handle = self._goal_future.result()
            if not self._goal_handle.accepted:
                self.node.get_logger().error("Goal rejected")
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # 2. Pre-rotate: cancel on first plan
        if not self._canceled_once:
            if self._saw_plan:
                self._goal_handle.cancel_goal_async()
                self.node.get_logger().warn("Plan received → canceled for pre-rotate")

                try:
                    yaw = self._yaw_from_plan(self._latest_plan, self.lookahead_idx)
                    self._pre_yaw_time = self._publish_target_yaw(yaw)
                    self.node.get_logger().info(f"Pre-rotate yaw: {yaw:.2f} rad")
                except Exception as e:
                    self.node.get_logger().warn(f"Pre-yaw failed: {e}")

                self._canceled_once = True
                self._saw_plan = False
                self._goal_handle = None
            return py_trees.common.Status.RUNNING

        # 3. Wait pre-align → resend
        if not self._resent:
            if self._pre_yaw_time is not None:
                wait = (now - self._pre_yaw_time).nanoseconds / 1e9
                if wait > self.align_timeout_sec:
                    self.node.get_logger().warn(f"Pre-align timeout ({wait:.1f}s) → resending anyway")
                elif not self._yaw_aligned:
                    return py_trees.common.Status.RUNNING

            self.node.get_logger().info("Pre-align done → resending goal")
            self._goal_future = self._client.send_goal_async(self._goal_msg)
            self._goal_handle = None
            self._resent = True
            self._yaw_aligned = False  # reset for final
            return py_trees.common.Status.RUNNING

        # 4. Wait for nav result
        if not self._result_future.done():
            return py_trees.common.Status.RUNNING

        result = self._result_future.result()
        status = result.status
        error_code = result.result.error_code

        if self.log_once:
            
            self.node.get_logger().info(f"Nav result: status={status}, error={error_code}")
            self.log_once = False

        if status != 4 or error_code != 0:
            return py_trees.common.Status.FAILURE

        # 5. Always final yaw correction (same as original goal yaw)
        if not self._final_yaw_published:
            self._final_yaw_time = self._publish_target_yaw(self._target_yaw)
            self.node.get_logger().info(f"Final correction yaw: {self._target_yaw:.2f} rad")
            self._final_yaw_published = True
            self._yaw_aligned = False
            
            return py_trees.common.Status.RUNNING

        # 6. Wait for final alignment
        wait = (now - self._final_yaw_time).nanoseconds / 1e9
        if wait > self.align_timeout_sec:
            
            self.node.get_logger().warn(f"Final yaw timeout ({wait:.1f}s) → success anyway")
                
            return py_trees.common.Status.SUCCESS

        if not self._yaw_aligned:
            return py_trees.common.Status.RUNNING

        self.node.get_logger().info("Final yaw aligned → success")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except:
                pass