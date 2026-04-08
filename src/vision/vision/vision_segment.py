import rclpy
from rclpy.node import Node 
from rclpy.time import Time
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformListener , TransformBroadcaster
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('target_frame', 'odom')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        

    def cloud_callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id
        self.get_logger().info(f"frame_id: '{source_frame}'")

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return

        corrected_cloud = do_transform_cloud(msg, transform)

        h = msg.height
        w = msg.width

        points_struct = np.array(list(point_cloud2.read_points(
            corrected_cloud,
            field_names=("x", "y", "z"),
            skip_nans=False
        )))

        points = np.stack(
            [points_struct['x'], points_struct['y'], points_struct['z']],
            axis=-1
        ).astype(np.float32)

        points = points.reshape((h, w, 3))
        z_img = points[:, :, 2]
        x_img = points[:, :, 0]
        y_img = points[:, :, 1]

        # normalize for visualization (avoid NaNs)
        def normalize(img):
            img = np.nan_to_num(img, nan=0.0)
            min_val = np.min(img)
            max_val = np.max(img)
            if max_val - min_val > 1e-6:
                img = (img - min_val) / (max_val - min_val)
            return (img * 255).astype(np.uint8)

        z_vis = normalize(z_img)
        x_vis = normalize(x_img)
        y_vis = normalize(y_img)

        
        cv2.imwrite('/tmp/z_img.png', z_vis)
        cv2.imwrite('/tmp/x_img.png', x_vis)
        cv2.imwrite('/tmp/y_img.png', y_vis)
        self.get_logger().info(f"Images Saved: z_img.png, x_img.png, y_img.png")
        self.get_logger().info(f"raw cloud h={msg.height} w={msg.width}")
        self.get_logger().info(f"corrected cloud h={corrected_cloud.height} w={corrected_cloud.width}")
        self.get_logger().info(f"[vision] points shape: {points.shape}")
        self.get_logger().info(f"[vision] first point: {points[0,0]}")
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


