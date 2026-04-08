#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
import numpy as np
import struct
import math

from cv_bridge import CvBridge


class KinectPointCloudNode(Node):
    def __init__(self):
        super().__init__('kinect_pointcloud_node')

        self.bridge = CvBridge()

        # Intrinsics (must match what you used in Webots)
        self.fx = 525.0   # placeholder, adjust later
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5

        self.depth_sub = self.create_subscription(
            Image,
            'kinect/depth/image_raw',
            self.depth_callback,
            10
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            'kinect/points',
            10
        )

        self.get_logger().info('KinectPointCloudNode ready (sub: kinect/depth/image_raw, pub: kinect/points)')

    def depth_callback(self, msg: Image):
        # Expecting 32FC1 depth image in meters
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        if depth is None:
            return

        dh, dw = depth.shape
        fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy

        points = []
        step = 2  # downsample to keep it light

        for v in range(0, dh, step):
            for u in range(0, dw, step):
                z = float(depth[v, u])
                if not (0.2 < z < 5.0):
                    continue

                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append((x, y, z))

        if not points:
            return

        # Build PointCloud2
        cloud = PointCloud2()
        cloud.header = msg.header           # same stamp + frame_id = kinect_link
        cloud.height = 1
        cloud.width = len(points)

        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False

        flat = [c for p in points for c in p]
        cloud.data = struct.pack('<' + 'f' * len(flat), *flat)

        self.cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = KinectPointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
