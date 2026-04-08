import py_trees
from py_trees.common import Access

from robotino_interfaces.srv import SetGrammar


class SetGrammarMode(py_trees.behaviour.Behaviour):
    """
    Calls /set_grammar_mode to switch Vosk decoding mode.

    Inputs (blackboard optional):
      - grammar_mode (str)           (optional if you pass mode in ctor)
      - grammar_phrases (list[str])  (optional, used for CUSTOM)

    Returns:
      - RUNNING until service responds
      - SUCCESS if ok
      - FAILURE otherwise
    """

    def __init__(self, name, node,
                 mode=None,
                 phrases=None,
                 service_name="/set_grammar_mode",
                 bb_mode_key="grammar_mode",
                 bb_phrases_key="grammar_phrases"):
        super().__init__(name)
        self.node = node
        self.service_name = service_name

        self._mode_param = mode
        self._phrases_param = phrases

        self.bb_mode_key = bb_mode_key
        self.bb_phrases_key = bb_phrases_key

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(key=self.bb_mode_key, access=Access.READ)
        self.bb.register_key(key=self.bb_phrases_key, access=Access.READ)

        self._cli = self.node.create_client(SetGrammar, self.service_name)
        self._future = None

    def initialise(self):
        # Resolve mode + phrases from ctor, else BB
        mode = self._mode_param or getattr(self.bb, self.bb_mode_key, None)
        phrases = self._phrases_param if self._phrases_param is not None else getattr(self.bb, self.bb_phrases_key, [])

        if not mode:
            self.node.get_logger().error("[SetGrammarMode] No mode provided (ctor or blackboard).")
            self._future = None
            return

        req = SetGrammar.Request()
        req.mode = str(mode)
        req.phrases = [str(p) for p in (phrases or [])]

        if not self._cli.wait_for_service(timeout_sec=0.2):
            self.node.get_logger().warn(f"[SetGrammarMode] Service not available: {self.service_name}")
            self._future = None
            return

        self.node.get_logger().info(f"[SetGrammarMode] Setting mode={req.mode} phrases={len(req.phrases)}")
        self._future = self._cli.call_async(req)

    def update(self):
        if self._future is None:
            return py_trees.common.Status.FAILURE

        if not self._future.done():
            return py_trees.common.Status.RUNNING

        try:
            resp = self._future.result()
        except Exception as e:
            self.node.get_logger().error(f"[SetGrammarMode] Service call failed: {e}")
            return py_trees.common.Status.FAILURE

        if resp.ok:
            self.node.get_logger().info(f"[SetGrammarMode] OK: {resp.message}")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().warn(f"[SetGrammarMode] FAIL: {resp.message}")
        return py_trees.common.Status.FAILURE
