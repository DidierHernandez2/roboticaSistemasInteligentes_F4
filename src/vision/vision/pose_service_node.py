#!/usr/bin/env python3
"""
POSE Detect Service (ROS 2)

Behavior:
- Uses Ultralytics YOLO pose model (COCO-17)
- ALWAYS returns 2D keypoints
- Returns 3D keypoints ONLY when depth is valid
- Missing keypoints => NaN
- Output is a FLAT float32 array
- Debug image published on /vision/pose_debug_image
- No TF, no map, no base_link, sensor-frame only
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from robotino_interfaces.srv import PoseDetect

from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np


class PoseServiceNode(Node):
    def __init__(self):
        super().__init__("pose_service_node")

        self.bridge = CvBridge()

        # ---- Parameters (same style as YOLO service) ----
        self.declare_parameter("image_topic", "/kinect/rgb/image_raw")
        self.declare_parameter("depth_topic", "/kinect/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/kinect/depth/camera_info")
        self.declare_parameter("model_path", "yolo11n-pose.pt")

        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        depth_info_topic = self.get_parameter("depth_info_topic").value
        model_path = self.get_parameter("model_path").value

        self.get_logger().info(f"[POSE] RGB topic   : {image_topic}")
        self.get_logger().info(f"[POSE] Depth topic : {depth_topic}")
        self.get_logger().info(f"[POSE] Model       : {model_path}")

        # ---- Cached data (simple, like YOLO service) ----
        self.latest_image_msg = None
        self.latest_depth_msg = None
        self.latest_cam_info = None

        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.create_subscription(Image, depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.cam_info_cb, 10)

        # ---- Debug image publisher ----
        self.debug_pub = self.create_publisher(Image, "/vision/pose_debug_image", 10)

        # ---- Model ----
        self.model = YOLO(model_path)

        # ---- Service ----
        self.create_service(PoseDetect, "/pose_detect", self.handle_pose_detect)

        self.get_logger().info("[POSE] Service ready: /pose_detect")

    # ==================================================
    # Callbacks
    # ==================================================
    def image_cb(self, msg: Image):
        self.latest_image_msg = msg

    def depth_cb(self, msg: Image):
        self.latest_depth_msg = msg

    def cam_info_cb(self, msg: CameraInfo):
        self.latest_cam_info = msg

    # ==================================================
    # Service handler
    # ==================================================
    def handle_pose_detect(self, request, response):

        # ---------- Guard ----------
        if self.latest_image_msg is None:
            response.success = False
            response.status = "no RGB image yet"
            response.keypoints = []
            response.num_people = 0
            response.used_3d = False
            return response

        # ---------- RGB ----------
        frame = self.bridge.imgmsg_to_cv2(
            self.latest_image_msg, desired_encoding="bgr8"
        )
        H, W, _ = frame.shape

        # ---------- Depth (optional, same logic as YOLO) ----------
        depth_ok = (
            request.want_3d
            and self.latest_depth_msg is not None
            and self.latest_cam_info is not None
        )

        if depth_ok:
            try:
                depth = self.bridge.imgmsg_to_cv2(
                    self.latest_depth_msg, desired_encoding="passthrough"
                )
            except Exception as e:
                self.get_logger().warn(f"[POSE] depth cv_bridge error: {e}")
                depth_ok = False

        if depth_ok and depth.ndim != 2:
            self.get_logger().warn("[POSE] Depth not single-channel")
            depth_ok = False

        # --- Intrinsics (same constants you used in YOLO) ---
        fx = 525.0
        fy = 525.0
        cx = 319.5
        cy = 239.5

        if depth_ok:
            if self.latest_depth_msg.encoding == "16UC1":
                Z = depth.astype(np.float32) / 1000.0
            else:
                Z = depth.astype(np.float32)

            Z[~np.isfinite(Z)] = np.nan
            Z[Z <= 0.0] = np.nan
        else:
            Z = None

        # ---------- Run pose ----------
        results = self.model(frame, verbose=False)
        result = results[0]

        if result.keypoints is None:
            response.success = True
            response.status = "no people"
            response.keypoints = []
            response.num_people = 0
            response.used_3d = False
            response.header = self.latest_image_msg.header
            return response

        kpts = result.keypoints.xy.cpu().numpy()    # [N,17,2]
        conf = result.keypoints.conf.cpu().numpy()  # [N,17]

        num_people = kpts.shape[0]
        stride = 3 if depth_ok else 2

        out = np.full((num_people, 17, stride), np.nan, dtype=np.float32)

        # ---------- Fill output ----------
        for i in range(num_people):
            for j in range(17):
                if conf[i, j] < request.min_confidence:
                    continue

                u = float(kpts[i, j, 0])
                v = float(kpts[i, j, 1])

                if not depth_ok:
                    out[i, j, 0] = u
                    out[i, j, 1] = v
                else:
                    uu = int(round(u))
                    vv = int(round(v))

                    if 0 <= uu < W and 0 <= vv < H:
                        z = Z[vv, uu]
                        if np.isfinite(z):
                            x = (u - cx) * z / fx
                            y = (v - cy) * z / fy
                            out[i, j, 0] = x
                            out[i, j, 1] = y
                            out[i, j, 2] = z

        # ---------- Debug image ----------
        if request.publish_debug:
            dbg = result.plot()
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            dbg_msg.header = self.latest_image_msg.header
            self.debug_pub.publish(dbg_msg)

        # ---------- Fill response ----------
        response.header = self.latest_image_msg.header
        response.keypoints = out.reshape(-1).tolist()
        response.num_people = num_people
        response.used_3d = depth_ok
        response.success = True
        response.status = "ok"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
