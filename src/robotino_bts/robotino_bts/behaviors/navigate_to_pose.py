#!/usr/bin/env python3
# robotino_bts/behaviors/navigate_to_pose.py

import math
import py_trees
from py_trees.common import Access

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigateToPoseFromBB(py_trees.behaviour.Behaviour):
    """
    Behaviour Tree node that sends a Nav2 NavigateToPose goal
    using (x, y, yaw) read from the blackboard.

    Blackboard (READ):
      - goal_x: float
      - goal_y: float
      - goal_yaw: float (radians, in frame_id)

    Nav2:
      - Action server name: /navigate_to_pose
      - Action type: nav2_msgs/action/NavigateToPose

    Result handling:
      - status == 4 and error_code == 0  -> SUCCESS
      - otherwise                        -> FAILURE

    There is an optional behaviour:
      - If accept_aborted_near_goal=True and Nav2 returns ABORTED,
        but distance_remaining from feedback < near_goal_tolerance,
        the behaviour returns SUCCESS (useful if Nav2 aborts but
        the robot is physically at the goal).
    """

    def __init__(
        self,
        name: str,
        node,
        frame_id: str = "map",
        timeout_sec: float = 60.0,
        near_goal_tolerance: float = 0.30,
        accept_aborted_near_goal: bool = False,
    ):
        super().__init__(name)
        self.node = node
        self.frame_id = frame_id
        self.timeout_sec = timeout_sec
        self.near_goal_tolerance = near_goal_tolerance
        self.accept_aborted_near_goal = accept_aborted_near_goal

        # Blackboard client (read-only)
        self.bb = py_trees.blackboard.Client(name="NavBT_Blackboard")
        self.bb.register_key("goal_x", Access.READ)
        self.bb.register_key("goal_y", Access.READ)
        self.bb.register_key("goal_yaw", Access.READ)

        # ROS 2 action client (note the leading slash)
        self._client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

        # Internal state machine
        self._state = "IDLE"
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._start_time = None

        # Feedback tracking (for optional near-goal logic)
        self._last_distance_remaining = None
        self._last_nav_time_sec = None

    # -------------------------------------------------------------------------
    # BT lifecycle
    # -------------------------------------------------------------------------
    def initialise(self):
        """
        Called once when the BT enters this node.
        Sets up the goal and sends it to Nav2.
        """
        self._state = "IDLE"
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._start_time = self.node.get_clock().now()
        self._last_distance_remaining = None
        self._last_nav_time_sec = None

        # Check action server
        if not self._client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error(
                "[NavBT] /navigate_to_pose action server not available"
            )
            self._state = "DONE_FAILURE"
            return

        # Read goal from blackboard
        x = float(getattr(self.bb, "goal_x", 0.0))
        y = float(getattr(self.bb, "goal_y", 0.0))
        yaw = float(getattr(self.bb, "goal_yaw", 0.0))

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        half = 0.5 * yaw
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(half)
        goal_msg.pose.pose.orientation.w = math.cos(half)

        self.node.get_logger().info(
            f"[NavBT] Sending Nav2 goal: "
            f"({x:.2f}, {y:.2f}, yaw={yaw:.2f}) in frame '{self.frame_id}'"
        )

        # Send goal asynchronously, with feedback callback
        self._send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._state = "WAIT_GOAL_RESPONSE"

    def update(self):
        """
        Called repeatedly while this node is ticked by the BT.

        Returns:
          - RUNNING while waiting for Nav2
          - SUCCESS when navigation completed successfully
          - FAILURE if rejected, aborted, canceled, or timeout
        """
        # Short debug: where are we in the small internal FSM?
        send_done = self._send_future.done() if self._send_future is not None else None
        result_done = self._result_future.done() if self._result_future is not None else None
        self.node.get_logger().debug(
            f"[NavBT DEBUG] state={self._state}, "
            f"send_done={send_done}, has_result_future={self._result_future is not None}, "
            f"result_done={result_done}"
        )

        # Terminal states
        if self._state == "DONE_FAILURE":
            return py_trees.common.Status.FAILURE
        if self._state == "DONE_SUCCESS":
            return py_trees.common.Status.SUCCESS

        # Global timeout
        if self._start_time is not None:
            now = self.node.get_clock().now()
            elapsed = (now.nanoseconds - self._start_time.nanoseconds) * 1e-9
            if elapsed > self.timeout_sec:
                self.node.get_logger().warn(
                    f"[NavBT] Timeout ({elapsed:.1f}s), treating navigation as FAILURE"
                )
                self._state = "DONE_FAILURE"
                return py_trees.common.Status.FAILURE

        # ---------------------------------------------------------------------
        # 1) Wait for goal to be accepted / rejected
        # ---------------------------------------------------------------------
        if self._state == "WAIT_GOAL_RESPONSE":
            if self._send_future is None:
                self.node.get_logger().error(
                    "[NavBT] _send_future is None in WAIT_GOAL_RESPONSE"
                )
                self._state = "DONE_FAILURE"
                return py_trees.common.Status.FAILURE

            if not self._send_future.done():
                return py_trees.common.Status.RUNNING

            # Goal response is ready
            self._goal_handle = self._send_future.result()
            if not self._goal_handle.accepted:
                self.node.get_logger().error("[NavBT] Goal rejected by Nav2")
                self._state = "DONE_FAILURE"
                return py_trees.common.Status.FAILURE

            self.node.get_logger().info("[NavBT] Goal accepted, waiting for result...")
            self._result_future = self._goal_handle.get_result_async()
            self._state = "WAIT_RESULT"
            return py_trees.common.Status.RUNNING

        # ---------------------------------------------------------------------
        # 2) Wait for Nav2 result (or near-goal fallback)
        # ---------------------------------------------------------------------
        if self._state == "WAIT_RESULT":
            # If result is ready, use it
            if self._result_future is not None and self._result_future.done():
                try:
                    outer = self._result_future.result()
                except Exception as e:
                    self.node.get_logger().error(
                        f"[NavBT] Exception getting result from Nav2: {e}"
                    )
                    self._state = "DONE_FAILURE"
                    return py_trees.common.Status.FAILURE

                # outer has .status (GoalStatus) and .result (NavigateToPose_Result)
                status = int(getattr(outer, "status", -1))
                result = getattr(outer, "result", None)
                error_code = getattr(result, "error_code", -1) if result is not None else -1
                error_msg = getattr(result, "error_msg", "") if result is not None else ""

                self.node.get_logger().info(
                    f"[NavBT] Nav2 result: status={status}, "
                    f"error_code={error_code}, msg='{error_msg}'"
                )

                # GoalStatus mapping:
                # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
                if status == 4 and error_code == 0:
                    self._state = "DONE_SUCCESS"
                    return py_trees.common.Status.SUCCESS

                # Optional: treat "ABORTED but very close to goal" as success
                if (
                    status == 6
                    and self.accept_aborted_near_goal
                    and self._last_distance_remaining is not None
                    and self._last_distance_remaining < self.near_goal_tolerance
                ):
                    self.node.get_logger().warn(
                        f"[NavBT] ABORTED but dist_rem="
                        f"{self._last_distance_remaining:.2f} < {self.near_goal_tolerance:.2f}, "
                        "treating as SUCCESS for BT flow."
                    )
                    self._state = "DONE_SUCCESS"
                    return py_trees.common.Status.SUCCESS

                # Otherwise, treat as failure
                if status == 5:
                    self.node.get_logger().warn("[NavBT] NavigateToPose CANCELED")
                elif status == 6:
                    self.node.get_logger().warn("[NavBT] NavigateToPose ABORTED")
                else:
                    self.node.get_logger().warn(
                        f"[NavBT] NavigateToPose finished with unexpected status={status}"
                    )
                self._state = "DONE_FAILURE"
                return py_trees.common.Status.FAILURE

            # If result is not yet ready, keep running
            return py_trees.common.Status.RUNNING

        # Fallback
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Called when the BT stops ticking this node (e.g., higher-level abort).
        We cancel the Nav2 goal if it is still active.
        """
        if self._goal_handle is not None and self._state in ("WAIT_GOAL_RESPONSE", "WAIT_RESULT"):
            self.node.get_logger().info("[NavBT] Cancelling navigation goal")
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as e:
                self.node.get_logger().warn(f"[NavBT] Exception while cancelling goal: {e}")

    # -------------------------------------------------------------------------
    # Feedback callback
    # -------------------------------------------------------------------------
    def _feedback_callback(self, feedback_msg):
        """
        Nav2 feedback callback. Stores distance_remaining and logs progress.
        """
        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose
        dist = float(feedback.distance_remaining)
        nav_time = feedback.navigation_time
        rec = feedback.number_of_recoveries

        t = nav_time.sec + nav_time.nanosec * 1e-9

        # Store for potential near-goal logic
        self._last_distance_remaining = dist
        self._last_nav_time_sec = t

        self.node.get_logger().info(
            f"[NavBT FEEDBACK] pos=({pose.position.x:.2f}, {pose.position.y:.2f}) "
            f"dist_rem={dist:.2f} recov={rec} t_nav={t:.1f}s"
        )
