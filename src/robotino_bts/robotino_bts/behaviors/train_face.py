# robotino_bts/behaviors/train_face.py
import py_trees
from py_trees.common import Access
from robotino_interfaces.srv import FaceRecog

class TrainFace(py_trees.behaviour.Behaviour):
    """
    Trains the face DB with the current camera image using a name from the blackboard.

    Blackboard (read):
      - <name_key> : str    (e.g., "current_guest_name")

    Calls:
      - /face_train (FaceRecog)

    Returns:
      - RUNNING while waiting for service response
      - SUCCESS if training happened (service returns name_response and/or "trained")
      - FAILURE otherwise
    """
    def __init__(self, name, node, name_key="current_guest_name", timeout_sec=3.0):
        super().__init__(name)
        self.node = node
        self.name_key = name_key
        self.timeout_sec = float(timeout_sec)

        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key(key=self.name_key, access=Access.READ)

        self.cli = self.node.create_client(FaceRecog, "/face_train")

        self._future = None
        self._start_time = None

    def initialise(self):
        self._future = None
        self._start_time = self.node.get_clock().now()

        # 1) get name
        train_name = (self.bb.get(self.name_key) or "").strip()
        if not train_name:
            self.node.get_logger().error(f"[TRAIN_FACE] Blackboard '{self.name_key}' is empty")
            return

        # 2) ensure service exists
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[TRAIN_FACE] /face_train not available")
            return

        # 3) call service (this is what actually scans the image + stores embedding)
        req = FaceRecog.Request()
        req.name_request = [train_name]
        req.min_confidence = 0.0  # your service ignores this currently, but ok

        self._future = self.cli.call_async(req)
        self.node.get_logger().info(f"[TRAIN_FACE] Requested training for '{train_name}'")

    def update(self):
        if self._future is None:
            return py_trees.common.Status.FAILURE

        # optional timeout (prevents hanging forever)
        if self._start_time is not None:
            dt = (self.node.get_clock().now() - self._start_time).nanoseconds * 1e-9
            if dt > self.timeout_sec:
                self.node.get_logger().error("[TRAIN_FACE] Timeout waiting for /face_train")
                return py_trees.common.Status.FAILURE

        if not self._future.done():
            return py_trees.common.Status.RUNNING

        try:
            resp = self._future.result()
        except Exception as e:
            self.node.get_logger().error(f"[TRAIN_FACE] /face_train call failed: {e}")
            return py_trees.common.Status.FAILURE

        # SUCCESS criteria: service indicates training happened
        if not resp.name_response:
            self.node.get_logger().info("[TRAIN_FACE] No training result (no face detected?)")
            return py_trees.common.Status.FAILURE

        # If you want to be strict, check resp.features contains "trained"
        if resp.features and ("trained" not in resp.features[0]):
            self.node.get_logger().warn(f"[TRAIN_FACE] Unexpected features: {resp.features}")

        self.node.get_logger().info(f"[TRAIN_FACE] Trained: {resp.name_response}")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # nothing to cancel in rclpy futures cleanly; just drop reference
        self._future = None
        self._start_time = None
