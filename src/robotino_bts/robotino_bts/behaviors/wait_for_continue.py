#!/usr/bin/env python3
import py_trees
from py_trees.common import Status
from py_trees.blackboard import Blackboard

import rclpy
from std_msgs.msg import Bool


class WaitForContinue(py_trees.behaviour.Behaviour):
    """
    Debug leaf:
      - logs selected blackboard keys every tick (or once)
      - returns RUNNING until a BB flag becomes True
      - you can toggle that flag by publishing Bool(True) on a topic

    Blackboard:
      continue_key (default): "debug_continue"
        - when True -> SUCCESS and resets to False (optional)
    """

    def __init__(
        self,
        name: str = "WaitForContinue",
        continue_key: str = "debug_continue",
        keys_to_print: list[str] | None = None,
        print_every_tick: bool = False,
        reset_on_success: bool = True,
        continue_topic: str = "/bt/continue",
    ):
        super().__init__(name)

        self.continue_key = continue_key
        self.keys_to_print = keys_to_print or []
        self.print_every_tick = print_every_tick
        self.reset_on_success = reset_on_success
        self.continue_topic = continue_topic

        self.bb = Blackboard()

        # Use a "client" so py_trees can enforce read/write access
        self.bb_client = self.attach_blackboard_client(name=name)
        self.bb_client.register_key(key=self.continue_key, access=py_trees.common.Access.WRITE)

        # Register read keys if specified
        for k in self.keys_to_print:
            self.bb_client.register_key(key=k, access=py_trees.common.Access.READ)

        self._printed_once = False
        self._node = None
        self._sub = None

    def setup(self, **kwargs):
        """
        Expect a ROS2 node passed in as: setup(node=<rclpy.node.Node>)
        Your BT executor already has a node; pass it here.
        """
        self._node = kwargs.get("node", None)
        if self._node is None:
            raise RuntimeError("WaitForContinue requires setup(node=<rclpy Node>)")

        # Initialize flag if missing
        if not hasattr(self.bb, self.continue_key):
            setattr(self.bb, self.continue_key, False)

        # Subscribe to topic to flip the continue flag
        self._sub = self._node.create_subscription(
            Bool,
            self.continue_topic,
            self._continue_cb,
            10
        )
        return True

    def _continue_cb(self, msg: Bool):
        if bool(msg.data):
            setattr(self.bb, self.continue_key, True)

    def initialise(self):
        self._printed_once = False

    def _dump_bb(self):
        # Print selected keys or the whole blackboard snapshot
        if self.keys_to_print:
            lines = []
            for k in self.keys_to_print:
                val = getattr(self.bb, k, "<unset>")
                lines.append(f"{k}={val}")
            self._node.get_logger().info("[BB] " + " | ".join(lines))
        else:
            # Print everything (best-effort)
            try:
                data = self.bb.__dict__
                self._node.get_logger().info("[BB] " + " | ".join([f"{k}={v}" for k, v in data.items()]))
            except Exception:
                self._node.get_logger().info("[BB] (could not dump full blackboard)")

    def update(self) -> Status:
        if self._node is None:
            return Status.FAILURE

        if (not self._printed_once) or self.print_every_tick:
            self._dump_bb()
            self._printed_once = True
            self._node.get_logger().info(
                f"[WaitForContinue] Paused. Publish Bool(True) to {self.continue_topic} to continue."
            )

        go = bool(getattr(self.bb, self.continue_key, False))
        if go:
            if self.reset_on_success:
                setattr(self.bb, self.continue_key, False)
            self._node.get_logger().info("[WaitForContinue] Continue received -> SUCCESS")
            return Status.SUCCESS

        return Status.RUNNING
