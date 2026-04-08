from typing import Callable, Union
import inspect
import py_trees
from robotino_interfaces.srv import Talk


TextOrCallable = Union[str, Callable[[], str], Callable[[object], str]]

class SayTextBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, text: TextOrCallable, wait: bool = True, service_name: str = "/tts/talk"):
        super().__init__(name)
        self.node = node
        self.text = text
        self.wait = wait
        self.service_name = service_name

        self._client = None
        self._future = None
        self._resolved_text = ""
        self._skip_speaking = False

    def _resolve_text(self) -> str:
        t = self.text
        if isinstance(t, str):
            return t
        if callable(t):
            try:
                sig = inspect.signature(t)
                if len(sig.parameters) == 0:
                    return str(t())
                return str(t(self.node))
            except Exception as e:
                self.node.get_logger().error(f"[SAY_TEXT] Failed to resolve dynamic text: {e}")
                return ""
        return ""

    def setup(self, **kwargs):
        if self._client is None:
            self._client = self.node.create_client(Talk, self.service_name)

    def initialise(self):
        # Resolve right before speaking (dynamic)
        self._resolved_text = (self._resolve_text() or "").strip()
        self._skip_speaking = (self._resolved_text == "")

        if self._skip_speaking:
            self._future = None
            return

        if self._client is None:
            self._client = self.node.create_client(Talk, self.service_name)

        if not self._client.wait_for_service(timeout_sec=0.2):
            self.node.get_logger().error(f"[SAY_TEXT] Service not available: {self.service_name}")
            self._future = None
            return

        req = Talk.Request()
        req.text = self._resolved_text
        req.wait = bool(self.wait)

        self.node.get_logger().info(f"[SAY_TEXT] Calling {self.service_name} wait={self.wait}: {self._resolved_text}")
        self._future = self._client.call_async(req)

    def update(self):
        if self._skip_speaking:
            return py_trees.common.Status.SUCCESS

        if self._future is None:
            return py_trees.common.Status.FAILURE

        if not self._future.done():
            return py_trees.common.Status.RUNNING

        try:
            resp = self._future.result()
        except Exception as e:
            self.node.get_logger().error(f"[SAY_TEXT] Service call failed: {e}")
            return py_trees.common.Status.FAILURE

        if getattr(resp, "success", False):
            return py_trees.common.Status.SUCCESS

        msg = getattr(resp, "message", "unknown error")
        self.node.get_logger().warn(f"[SAY_TEXT] TTS returned failure: {msg}")
        return py_trees.common.Status.FAILURE
