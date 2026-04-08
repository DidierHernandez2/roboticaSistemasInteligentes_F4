#!/usr/bin/env python3
"""
Continuous person detection node.
Calls /yolo_detect service on a timer and displays annotated results.
Rotates the robot until a person is detected, then stops.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robotino_interfaces.srv import YoloDetect
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2


class YoloPersonNode(Node):
    def __init__(self):
        super().__init__('yolo_person_node')

        self.declare_parameter('display', True)
        self.declare_parameter('interval', 0.3)  # seconds between detections
        self.declare_parameter('spin_speed', 0.4)  # rad/s
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        interval = self.get_parameter('interval').get_parameter_value().double_value
        self.spin_speed = self.get_parameter('spin_speed').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.client = self.create_client(YoloDetect, '/yolo_detect')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._person_found = False

        # Subscribe to debug image published by yolo_server
        self.debug_sub = self.create_subscription(
            Image, '/vision/yolo_debug_image', self.debug_callback, 10
        )

        self.timer = self.create_timer(interval, self.detect)
        self._waiting = False

        if self.display:
            cv2.namedWindow('YOLO Person Detection', cv2.WINDOW_NORMAL)

        self.get_logger().info(f'[yolo_person] Calling /yolo_detect every {interval}s. Spinning at {self.spin_speed} rad/s until person found. Press q to quit.')

    def _publish_spin(self):
        msg = Twist()
        msg.angular.z = self.spin_speed
        self.cmd_vel_pub.publish(msg)

    def _publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def detect(self):
        if not self.client.service_is_ready():
            self.get_logger().warn('[yolo_person] Waiting for /yolo_detect service...')
            return

        if self._waiting:
            return

        # Keep spinning while no person found
        if not self._person_found:
            self._publish_spin()

        req = YoloDetect.Request()
        future = self.client.call_async(req)
        self._waiting = True
        future.add_done_callback(self.on_result)

    def on_result(self, future):
        self._waiting = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'[yolo_person] Service call failed: {e}')
            return

        # Filter only persons (class "person")
        persons = [
            (name, det)
            for name, det in zip(resp.class_names, resp.detections.detections)
            if name == 'person'
        ]

        if persons and not self._person_found:
            self._person_found = True
            self._publish_stop()
            self.get_logger().info(f'[yolo_person] Person found! Stopping rotation.')
        elif persons:
            self.get_logger().info(f'[yolo_person] Detected {len(persons)} person(s)')

    def debug_callback(self, msg: Image):
        if not self.display:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('YOLO Person Detection', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('[yolo_person] Quitting...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().warn(f'[yolo_person] Display error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonNode()
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
