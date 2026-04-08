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

        
    

        # Store last debug image (generic)
        self.latest_debug_cv = None
        self.latest_debug_source = ""

        

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
    #def yolo_debug_image_callback(self, msg: Image):
    #    self._show_debug(msg, "yolo")

    # ======================================
    # FACE DEBUG IMAGE CALLBACK
    # ======================================
    #def face_debug_image_callback(self, msg: Image):
    #    self._show_debug(msg, "face")

    # ======================================
    # POSE DEBUG IMAGE CALLBACK
    # ======================================
    #def pose_debug_image_callback(self, msg: Image):
    #    self._show_debug(msg, "pose")


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
