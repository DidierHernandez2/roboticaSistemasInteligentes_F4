import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robotino_interfaces.srv import YoloDetect, FaceRecog, PoseDetect

import cv2
from cv_bridge import CvBridge


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- Parameter for main RGB image topic ---
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.get_logger().info(f'[vision] Using image_topic: {image_topic}')

        self.bridge = CvBridge()

        # --- Subscribe to RGB camera ---
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # --- Subscribe to YOLO debug image topic ---
        self.yolo_debug_sub = self.create_subscription(
            Image,
            '/vision/yolo_debug_image',
            self.yolo_debug_image_callback,
            10
        )

        # --- Subscribe to FACE debug image topic ---
        self.face_debug_sub = self.create_subscription(
            Image,
            '/vision/face_recog_debug_image',
            self.face_debug_image_callback,
            10
        )

        # --- Subscribe to POSE debug image topic ---
        self.pose_debug_sub = self.create_subscription(
            Image,
            '/vision/pose_debug_image',
            self.pose_debug_image_callback,
            10
        )

        # Store last debug image (generic)
        self.latest_debug_cv = None
        self.latest_debug_source = ""

        # --- YOLO Detect service client (on-demand) ---
        self.yolo_client = self.create_client(YoloDetect, 'yolo_detect')
        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("[vision] Waiting for yolo_detect service...")
        self.yolo_call_in_flight = False

        # --- FACE Recog service client (on-demand) ---
        self.face_client = self.create_client(FaceRecog, 'face_recog')
        while not self.face_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("[vision] Waiting for face_recog service...")
        self.face_call_in_flight = False

        # --- POSE Detect service client (on-demand) ---
        self.pose_client = self.create_client(PoseDetect, 'pose_detect')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("[vision] Waiting for pose_detect service...")
        self.pose_call_in_flight = False

        # --- OpenCV windows ---
        cv2.namedWindow('vision', cv2.WINDOW_NORMAL)
        cv2.namedWindow('debug_image', cv2.WINDOW_NORMAL)

        self.get_logger().info("[vision] Keys: q=quit, y=YOLO, f=FACE, p=POSE")

    # ======================================
    # MAIN RGB CALLBACK
    # ======================================
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        cv2.imshow('vision', cv_image)
        key = cv2.waitKey(1) & 0xFF

        # Quit
        if key == ord('q'):
            self.get_logger().info("Received 'q' – shutting down vision_node.")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        # Trigger YOLO once when pressing 'y'
        if key == ord('y') and not self.yolo_call_in_flight:
            self.get_logger().info("[vision] Calling YOLO service...")
            self.yolo_call_in_flight = True

            yolo_req = YoloDetect.Request()
            yolo_call = self.yolo_client.call_async(yolo_req)
            yolo_call.add_done_callback(self.handle_yolo_response)

        # Trigger FACE once when pressing 'f'
        if key == ord('f') and not self.face_call_in_flight:
            self.get_logger().info("[vision] Calling FACE service...")
            self.face_call_in_flight = True

            face_req = FaceRecog.Request()
            face_req.name_request = []
            face_req.min_confidence = 0.0

            face_call = self.face_client.call_async(face_req)
            face_call.add_done_callback(self.handle_face_response)

        # Trigger POSE once when pressing 'p'
        if key == ord('p') and not self.pose_call_in_flight:
            self.get_logger().info("[vision] Calling POSE service...")
            self.pose_call_in_flight = True

            req = PoseDetect.Request()
            req.name_request = []
            req.min_confidence = 0.3
            req.want_3d = True
            req.publish_debug = True

            future = self.pose_client.call_async(req)

            def _pose_done(fut):
                self.pose_call_in_flight = False
                try:
                    resp = fut.result()
                except Exception as e:
                    self.get_logger().error(f"[vision] POSE service call failed: {e}")
                    return

                if not resp.success:
                    self.get_logger().warn(f"[vision] POSE failed: {resp.status}")
                    return

                self.get_logger().info(
                    f"[vision] POSE returned {resp.num_people} people "
                    f"(3D={'yes' if resp.used_3d else 'no'})"
                )

            future.add_done_callback(_pose_done)

    # ======================================
    # YOLO SERVICE RESPONSE
    # ======================================
    def handle_yolo_response(self, yolo_call):
        self.yolo_call_in_flight = False
        try:
            yolo_resp = yolo_call.result()
        except Exception as e:
            self.get_logger().error(f"[vision] YOLO service call failed: {e}")
            return

        try:
            n = len(yolo_resp.detections.detections)
        except Exception:
            n = 0
        self.get_logger().info(f"[vision] YOLO returned {n} detections.")

    # ======================================
    # FACE SERVICE RESPONSE
    # ======================================
    def handle_face_response(self, face_call):
        self.face_call_in_flight = False
        try:
            face_resp = face_call.result()
        except Exception as e:
            self.get_logger().error(f"[vision] FACE service call failed: {e}")
            return

        try:
            n = len(face_resp.name_response)
        except Exception:
            n = 0
        self.get_logger().info(f"[vision] FACE returned {n} faces.")

    # ======================================
    # GENERIC DEBUG SHOW
    # ======================================
    def _show_debug(self, msg: Image, source: str):
        try:
            debug_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'[vision] Failed to convert {source} debug image: {e}')
            return

        self.latest_debug_cv = debug_cv
        self.latest_debug_source = source

        try:
            cv2.setWindowTitle('debug_image', f'debug_image ({source})')
        except Exception:
            pass

        cv2.imshow('debug_image', self.latest_debug_cv)
        cv2.waitKey(1)

    # ======================================
    # YOLO DEBUG IMAGE CALLBACK
    # ======================================
    def yolo_debug_image_callback(self, msg: Image):
        self._show_debug(msg, "yolo")

    # ======================================
    # FACE DEBUG IMAGE CALLBACK
    # ======================================
    def face_debug_image_callback(self, msg: Image):
        self._show_debug(msg, "face")

    # ======================================
    # POSE DEBUG IMAGE CALLBACK
    # ======================================
    def pose_debug_image_callback(self, msg: Image):
        self._show_debug(msg, "pose")


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
