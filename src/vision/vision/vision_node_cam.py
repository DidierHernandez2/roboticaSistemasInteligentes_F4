import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robotino_interfaces.srv import YoloDetect

import cv2
from cv_bridge import CvBridge

## VENV HACK #!/home/oscar/venvs/yolo_env/bin/python3##
###/home/oscar/robotino_ros2_ws/install/vision/lib/vision/kinect_pointcloud_node

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
        # Change this if your topic is named differently
        self.debug_sub = self.create_subscription(
            Image,
            '/yolo_debug_image',
            self.debug_image_callback,
            10
        )

        # Store last debug image
        self.latest_debug_cv = None

        # --- YOLO Detect service client (on-demand) ---
        self.yolo_client = self.create_client(YoloDetect, 'yolo_detect')
        while not self.yolo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("[vision] Waiting for /yolo_detect service...")

        self.call_in_flight = False

        # --- OpenCV windows ---
        cv2.namedWindow('vision', cv2.WINDOW_NORMAL)
        cv2.namedWindow('yolo_debug', cv2.WINDOW_NORMAL)

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
        if key == ord('y') and not self.call_in_flight:
            self.get_logger().info("[vision] Calling YOLO service...")
            self.call_in_flight = True
            req = YoloDetect.Request()
            future = self.yolo_client.call_async(req)
            future.add_done_callback(self.handle_yolo_response)

    # ======================================
    # YOLO SERVICE RESPONSE
    # ======================================
    def handle_yolo_response(self, future):
        self.call_in_flight = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"[vision] YOLO service call failed: {e}")
            return

        # Just log number of detections
        try:
            n = len(resp.detections.detections)
        except Exception:
            n = 0
        self.get_logger().info(f"[vision] YOLO returned {n} detections.")

        # The debug image itself comes via the /vision/yolo_debug_image topic.
        # When the YOLO node publishes it, debug_image_callback will update the window.

    # ======================================
    # DEBUG IMAGE CALLBACK (from YOLO topic)
    # ======================================
    def debug_image_callback(self, msg: Image):
        try:
            # Use the encoding you set in the YOLO node: 'bgr8' or 'rgb8'
            debug_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'[vision] Failed to convert debug image: {e}')
            return

        self.latest_debug_cv = debug_cv

        # Show it immediately when received
        cv2.imshow('yolo_debug', self.latest_debug_cv)
        cv2.waitKey(1)


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
