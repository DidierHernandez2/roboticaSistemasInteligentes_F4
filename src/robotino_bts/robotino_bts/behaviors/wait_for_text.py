import py_trees
from py_trees.common import Access
from std_msgs.msg import String


class WaitForText(py_trees.behaviour.Behaviour):
    """
    Waits for a *new* message on /speech/text.

    Blackboard outputs:
      - speech_last_text (str)
      - speech_last_stamp_ns (int)

    Returns:
      - RUNNING until a new message arrives after initialise()
      - SUCCESS once it stores the new message
    """

    def __init__(self, name, node, topic="/speech/text",
                 bb_text_key="speech_last_text",
                 bb_stamp_key="speech_last_stamp_ns"):
        super().__init__(name)
        self.node = node
        self.topic = topic
        self.bb_text_key = bb_text_key
        self.bb_stamp_key = bb_stamp_key

        # Blackboard
        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(key=self.bb_text_key, access=Access.WRITE)
        self.bb.register_key(key=self.bb_stamp_key, access=Access.WRITE)

        # Internal cache
        self._latest_text = None
        self._latest_stamp_ns = None
        self._start_stamp_ns = None

        # Subscribe once (infrastructure)
        self._sub = self.node.create_subscription(
            String,
            self.topic,
            self._cb,
            10
        )

    def _cb(self, msg: String):
        # Cache latest text + time received (receipt time is good enough)
        self._latest_text = msg.data
        self._latest_stamp_ns = self.node.get_clock().now().nanoseconds

    def initialise(self):
        # Freshness boundary: only accept speech that arrives AFTER this moment
        self._start_stamp_ns = self.node.get_clock().now().nanoseconds
        self.node.get_logger().info(f"[WAIT_FOR_TEXT] TALK NOW'")

    def update(self):
        if self._latest_text is None or self._latest_stamp_ns is None:
            return py_trees.common.Status.RUNNING

        # Ignore old messages (received before we started waiting)
        if self._latest_stamp_ns <= self._start_stamp_ns:
            return py_trees.common.Status.RUNNING

        # Write to BB and succeed
        self.bb.set(self.bb_text_key, self._latest_text)
        self.bb.set(self.bb_stamp_key, int(self._latest_stamp_ns))

        self.node.get_logger().info(f"[WAIT_FOR_TEXT] Heard: '{self._latest_text}'")
        return py_trees.common.Status.SUCCESS
