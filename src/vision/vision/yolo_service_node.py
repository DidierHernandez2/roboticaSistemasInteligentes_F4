#!/usr/bin/env python3
"""
YOLO Detect Service (ROS 2) — corrected drop-in version

Key behavior:
- ALWAYS returns 2D detections + class_names.
- Returns 3D PoseStamped ONLY when depth is valid:
    pose.header.frame_id = parent_frame (typically your depth frame)
    pose.pose.position = (cx3, cy3, cz3) in that frame
- DOES NOT depend on map / base_link / localization.
- DOES NOT do any TF lookups to map.
- Publishes TF frames yolo_<class>_<i> as child of parent_frame (sensor frame).
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from robotino_interfaces.srv import YoloDetect

from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped


class YoloServiceNode(Node):
    def __init__(self):
        super().__init__("yolo_service_node")

        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameter("image_topic", "/kinect/rgb/image_raw")
        self.declare_parameter("depth_topic", "/kinect/depth/image_raw")
        self.declare_parameter("depth_info_topic", "/kinect/depth/camera_info")
        self.declare_parameter("model_path", "yolo11n-seg.pt")  # adjust as needed

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        depth_info_topic = self.get_parameter("depth_info_topic").get_parameter_value().string_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        self.get_logger().info(f"[YOLO_SERVICE] Using RGB topic        : {image_topic}")
        self.get_logger().info(f"[YOLO_SERVICE] Using DEPTH topic      : {depth_topic}")
        self.get_logger().info(f"[YOLO_SERVICE] Using DEPTH info topic : {depth_info_topic}")
        self.get_logger().info(f"[YOLO_SERVICE] Using model            : {model_path}")
        self.get_logger().info("[YOLO_SERVICE] Output poses are in SENSOR frame only (no map/base).")

        # --- Subscriptions ---
        self.latest_image_msg = None
        self.latest_depth_msg = None
        self.latest_cam_info = None

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, depth_info_topic, self.cam_info_callback, 10)

        # --- TF broadcaster (publishes yolo_<class>_<i> under parent_frame) ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Debug image publisher ---
        self.debug_pub = self.create_publisher(Image, "/vision/yolo_debug_image", 10)

        # --- YOLO seg model ---
        self.model = YOLO(model_path)
        self.class_names = self.model.names  # dict: id -> name

        # --- Service server ---
        self.srv = self.create_service(YoloDetect, "/yolo_detect", self.handle_yolo_detect)

        self.get_logger().info("[YOLO_SERVICE] Ready (depth optional: 2D always, 3D+TF only when depth OK).")

    # ==========================
    # Callbacks
    # ==========================
    def image_callback(self, msg: Image):
        self.latest_image_msg = msg

    def depth_callback(self, msg: Image):
        self.latest_depth_msg = msg

    def cam_info_callback(self, msg: CameraInfo):
        self.latest_cam_info = msg

    # ==========================
    # Service handler
    # ==========================
    def handle_yolo_detect(self, request, response: YoloDetect.Response):
        # Always initialize
        response.class_names = []
        response.poses = []
        response.detections = Detection2DArray()

        if self.latest_image_msg is None:
            self.get_logger().warn("[YOLO_SERVICE] No RGB image yet.")
            return response

        # --- RGB image ---
        frame_bgr = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding="bgr8")
        H_img, W_img, _ = frame_bgr.shape

        # Prepare detections msg header
        detections_msg = Detection2DArray()
        detections_msg.header = self.latest_image_msg.header

        # --------------------------
        # Depth is OPTIONAL
        # --------------------------
        depth_ok = (self.latest_depth_msg is not None)
        if not depth_ok:
            self.get_logger().warn("[YOLO_SERVICE] No depth image yet. Returning 2D only (poses invalid).")

        X = Y = Z = None  # 3D arrays only if depth_ok
        parent_frame = "kinect_depth"
        if self.latest_depth_msg is not None and self.latest_depth_msg.header.frame_id:
            parent_frame = self.latest_depth_msg.header.frame_id

        if depth_ok:
            try:
                depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding="passthrough")
            except Exception as e:
                self.get_logger().warn(f"[YOLO_SERVICE] cv_bridge depth error: {e}")
                depth_ok = False

        if depth_ok:
            if depth.ndim != 2:
                self.get_logger().warn("[YOLO_SERVICE] Depth image is not single-channel. Returning 2D only.")
                depth_ok = False

        if depth_ok:
            H_d, W_d = depth.shape
            if H_d != H_img or W_d != W_img:
                self.get_logger().warn(
                    f"[YOLO_SERVICE] Depth size ({H_d},{W_d}) != RGB size ({H_img},{W_img}); returning 2D only."
                )
                depth_ok = False

        if depth_ok:
            # --- Intrinsics (keep your constants) --- pINHOLE model with typical Kinect parameters (adjust if you have real intrinsics)
            fx = 525.0
            fy = 525.0
            cx = 319.5
            cy = 239.5

            # --- Depth units ---
            if self.latest_depth_msg.encoding == "16UC1":
                Z = depth.astype(np.float32) / 1000.0  # mm -> m
            else:
                Z = depth.astype(np.float32)

            # Sanitize + clamp
            Z[~np.isfinite(Z)] = np.nan
            Z[Z <= 0.0] = np.nan
            Z[(Z < 0.25) | (Z > 6.0)] = np.nan

            # Back-project to 3D in the depth frame convention used by your projection
            u = np.tile(np.arange(W_img), (H_img, 1))
            v = np.tile(np.arange(H_img).reshape(-1, 1), (1, W_img))

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

        # --- YOLO segmentation ---
        results = self.model(frame_bgr)
        result = results[0]

        boxes = result.boxes
        masks = result.masks

        if boxes is None or len(boxes) == 0:
            self.get_logger().info("[YOLO_SERVICE] No detections.")
            annotated = result.plot()
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = self.latest_image_msg.header
            self.debug_pub.publish(debug_msg)

            response.detections = detections_msg
            return response

        boxes_xyxy = boxes.xyxy.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy().astype(int)
        confidences = boxes.conf.cpu().numpy()

        # Debug image
        annotated = result.plot()
        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        debug_msg.header = self.latest_image_msg.header
        self.debug_pub.publish(debug_msg)

        # Masks
        mask_array = None
        if masks is not None and masks.data is not None:
            mask_array = masks.data.cpu().numpy()  # [N, H, W]
            if mask_array.shape[1] != H_img or mask_array.shape[2] != W_img:
                self.get_logger().warn(
                    f"[YOLO_SERVICE] Mask size {mask_array.shape[1:]} does not match image size {(H_img, W_img)}."
                )
                mask_array = None

        now = self.get_clock().now().to_msg()

        # ==========================
        # Loop detections (2D always)
        # ==========================
        for i, box in enumerate(boxes_xyxy):
            detection = Detection2D()

            x1, y1, x2, y2 = box
            x1_i = max(0, min(W_img - 1, int(x1)))
            x2_i = max(0, min(W_img - 1, int(x2)))
            y1_i = max(0, min(H_img - 1, int(y1)))
            y2_i = max(0, min(H_img - 1, int(y2)))

            # 2D bbox
            bbox = BoundingBox2D()
            bbox.center.position.x = float((x1_i + x2_i) / 2.0)
            bbox.center.position.y = float((y1_i + y2_i) / 2.0)
            bbox.size_x = float(x2_i - x1_i)
            bbox.size_y = float(y2_i - y1_i)
            detection.bbox = bbox

            # hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_ids[i])
            hypothesis.hypothesis.score = float(confidences[i])
            detection.results = [hypothesis]

            detections_msg.detections.append(detection)

            # class name aligned with detections
            class_name = self.class_names.get(class_ids[i], "obj")
            response.class_names.append(class_name)

            # Default pose entry: invalid (no 3D)
            pose_out = PoseStamped()
            pose_out.header.frame_id = ""  # invalid unless we set 3D below

            child_frame_id = f"yolo_{class_name}_{i}"

            # 3D centroid + TF ONLY if depth is OK and mask available
            if (
                depth_ok
                and (mask_array is not None)
                and (i < mask_array.shape[0])
                and (X is not None) and (Y is not None) and (Z is not None)
            ):
                mask = mask_array[i]  # [H, W]
                submask = mask[y1_i:y2_i, x1_i:x2_i] > 0.5

                if np.any(submask):
                    ys_idx, xs_idx = np.where(submask)
                    ys_full = ys_idx + y1_i
                    xs_full = xs_idx + x1_i

                    X_pts = X[ys_full, xs_full]
                    Y_pts = Y[ys_full, xs_full]
                    Z_pts = Z[ys_full, xs_full]

                    good = np.isfinite(X_pts) & np.isfinite(Y_pts) & np.isfinite(Z_pts)

                    if np.any(good):
                        cx3 = float(np.nanmedian(X_pts[good]))
                        cy3 = float(np.nanmedian(Y_pts[good]))
                        cz3 = float(np.nanmedian(Z_pts[good]))

                        if np.isfinite(cx3) and np.isfinite(cy3) and np.isfinite(cz3):
                            # Publish TF (sensor-frame -> object)
                            t = TransformStamped()
                            t.header.stamp = now
                            t.header.frame_id = parent_frame
                            t.child_frame_id = child_frame_id
                            t.transform.translation.x = cx3
                            t.transform.translation.y = cy3
                            t.transform.translation.z = cz3
                            t.transform.rotation.x = 0.0
                            t.transform.rotation.y = 0.0
                            t.transform.rotation.z = 0.0
                            t.transform.rotation.w = 1.0
                            self.tf_broadcaster.sendTransform(t)

                            # Return PoseStamped in SENSOR frame (parent_frame)
                            pose_out = PoseStamped()
                            pose_out.header.stamp = now
                            pose_out.header.frame_id = parent_frame
                            pose_out.pose.position.x = cx3
                            pose_out.pose.position.y = cy3
                            pose_out.pose.position.z = cz3
                            pose_out.pose.orientation.x = 0.0
                            pose_out.pose.orientation.y = 0.0
                            pose_out.pose.orientation.z = 0.0
                            pose_out.pose.orientation.w = 1.0

            response.poses.append(pose_out)

        response.detections = detections_msg
        self.get_logger().info(
            f"[YOLO_SERVICE] Request handled: {len(detections_msg.detections)} detections, "
            f"3D={'yes' if depth_ok else 'no'} (poses valid only when 3D), "
            f"class_names={len(response.class_names)} poses={len(response.poses)}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
