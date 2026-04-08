# robotino_bts/behaviors/face_recognition.py
import py_trees
from py_trees.common import Access
from robotino_interfaces.srv import FaceRecog

class FaceRecognitionBehaviour(py_trees.behaviour.Behaviour):
    """
    Face recognition BT leaf with reactive semantics:

      - SUCCESS  : at least one face recognized
      - RUNNING  : waiting / no faces / service missing / transient errors
      - FAILURE  : NEVER (handled by parent Timeout if needed)
    """

    def __init__(self, name, node, guest_key="guest_1"):
        super().__init__(name)
        self.node = node
        self.guest_key = guest_key

        # Blackboard
        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(key=self.guest_key, access=Access.WRITE)
        self.bb.register_key("current_guest_name", access=Access.WRITE)

        # ROS2 service
        self.cli = self.node.create_client(FaceRecog, "/face_recog")

        self._future = None

    def initialise(self):
        self._future = None

    def update(self):
        # --- Ensure request in flight ---
        if self._future is None:
            if not self.cli.service_is_ready():
                # Service missing is NOT a failure for a reactive leaf
                return py_trees.common.Status.RUNNING

            req = FaceRecog.Request()
            req.name_request = []
            req.min_confidence = 0.0
            self._future = self.cli.call_async(req)
            return py_trees.common.Status.RUNNING

        # --- Waiting for response ---
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # --- Process response ---
        try:
            response = self._future.result()
        except Exception:
            # Transient service error → just try again
            self._future = None
            return py_trees.common.Status.RUNNING

        names = response.name_response or []

        # --- No faces → keep waiting ---
        if len(names) == 0:
            self._future = None
            return py_trees.common.Status.RUNNING

        # --- Write to blackboard ---
        guest_list = self.bb.get(self.guest_key) if self.bb.exists(self.guest_key) else []
        if guest_list is None:
            guest_list = []

        guest_list.extend(names)
        self.bb.set(self.guest_key, guest_list)

        # Pick first non-unknown from THIS response
        for n in names:
            if n and n.lower() != "unknown":
                self.bb.set("current_guest_name", n)
                break

        self.node.get_logger().info(
            f"[FACE_BT] Recognized {len(names)} face(s): {', '.join(names)}"
        )

        self._future = None
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._future = None
