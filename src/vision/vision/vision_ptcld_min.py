import rclpy
from rclpy.node import Node 
from rclpy.time import Time


from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- Parameter for point cloud topic ---
        self.declare_parameter('cloud_topic', '/kinect/points')
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.get_logger().info(f'[vision] Using cloud_topic: {cloud_topic}')

        # --- Subscribe to PointCloud2 ---
        self.subscription = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            10
        )

        self.get_logger().info("[vision] Waiting for point cloud...")

    # ======================================
    # POINT CLOUD CALLBACK
    # ======================================
    def cloud_callback(self, msg: PointCloud2):
        # Read points (x, y, z)
        points = list(point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            reshape_organized_cloud=True,
#           skip_nans=True
        ))

        # Convert to numpy
        points = np.array(points)

        # Just inspect
        self.get_logger().info(f"[vision] num points: {points.shape}")

        if len(points) > 0:
            self.get_logger().info(f"[vision] first point: {points[0]}")
            self.get_logger().info("[vision] PointCloud2 message received")


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


