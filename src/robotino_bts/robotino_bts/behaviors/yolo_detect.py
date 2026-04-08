# robotino_bts/behaviors/yolo_detect.py

import py_trees
from py_trees.common import Access

from robotino_interfaces.srv import YoloDetect


class YoloDetectBehaviour(py_trees.behaviour.Behaviour):
    """
    Calls /yolo_detect asynchronously.

    Stores in blackboard.detections_log a compact entry:
      {
        "classes":   [str, ...],
        "poses_kinect_link": [PoseStamped, ...],
      }
    """

    def __init__(self, name, node, service_name="yolo_detect"):
        super().__init__(name)
        self.node = node

        # Blackboard client
        self.bb = py_trees.blackboard.Client(name=f"{name}_BB")
        self.bb.register_key("detections_log", Access.WRITE)

        # ROS2 service client
        self.cli = self.node.create_client(YoloDetect, service_name)
        self.future = None
        self._done = False

    def initialise(self):
        self._done = False
        self.future = None

        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("[YOLO_BT] Service /yolo_detect not available")
            self._done = True
            return

        req = YoloDetect.Request()  # empty request
        self.future = self.cli.call_async(req)
        self.node.get_logger().info("[YOLO_BT] Request sent to /yolo_detect")

    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS

        if self.future is None:
            return py_trees.common.Status.RUNNING#FAILURE

        if not self.future.done():
            return py_trees.common.Status.RUNNING

        try:
            resp = self.future.result()
        except Exception as e:
            self.node.get_logger().error(f"[YOLO_BT] Service call failed: {e}")
            self._done = True
            return py_trees.common.Status.FAILURE

        classes = list(getattr(resp, "class_names", []) or [])
        poses_kinect_link = list(getattr(resp, "poses", []) or [])

        # Keep alignment safe (in case server returns mismatched lengths)
        n = min(len(classes), len(poses_kinect_link))
        classes = classes[:n]
        poses_kinect_link = poses_kinect_link[:n]

        log = self.bb.detections_log or []
        log.append({
            "classes": classes,
            "poses": poses_kinect_link,
        })
        self.bb.detections_log = log

        self.node.get_logger().info(f"[YOLO_BT] classes={classes}")

        self._done = True
        return py_trees.common.Status.SUCCESS
