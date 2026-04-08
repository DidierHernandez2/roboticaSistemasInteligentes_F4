#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')

        # --- TF odom->base_link ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Laser subscriber ---
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Log throttling
        self.msg_count = 0

        # History for delta tests
        self.last_x = None
        self.last_y = None
        self.last_front_range = None

        self.get_logger().info("line_detector (odom vs laser) initialized.")

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        Return (x,y,yaw) of robot in the ODOMETRY frame (odom->base_link)
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(f"No TF odom->base_link: {e}")
            return None

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        return x, y, yaw

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------------------------------------------------------
    # You can replace this with your real line-fitting code
    # ---------------------------------------------------------
    def compute_line(self, scan: LaserScan):
        ranges = list(scan.ranges)
        n = len(ranges)
        if n == 0:
            return None

        pts = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > math.radians(30):
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            pts.append((x, y))

        if len(pts) < 5:
            return None

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        mx = sum(xs)/len(xs)
        my = sum(ys)/len(ys)

        sxx = sum((x-mx)**2 for x in xs)
        syy = sum((y-my)**2 for y in ys)
        sxy = sum((x-mx)*(y-my) for x, y in pts)

        if sxx == syy:
            theta = 0.0
        else:
            theta = 0.5 * math.atan2(2*sxy, (sxx - syy))

        # signed distance
        nx = -math.sin(theta)
        ny = math.cos(theta)
        d = nx*mx + ny*my

        return theta, d

    # ---------------------------------------------------------
    # Main scan callback
    # ---------------------------------------------------------
    def scan_callback(self, scan: LaserScan):
        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        line = self.compute_line(scan)
        if line is None:
            return

        line_angle, d = line

        # grab front beam samples
        ranges = scan.ranges
        n = len(ranges)
        if n == 0:
            return

        c = n // 2
        offset = 5

        def safe(i): return max(0, min(n-1, i))

        r_left = ranges[safe(c - offset)]
        r_center = ranges[safe(c)]
        r_right = ranges[safe(c + offset)]

        # throttle (1 of 20)
        self.msg_count += 1
        if self.msg_count % 20 != 0:
            return

        odom_delta = None
        range_delta = None

        if (self.last_x is not None and
            self.last_y is not None and
            self.last_front_range is not None and
            math.isfinite(r_center)):

            dx = x - self.last_x
            dy = y - self.last_y
            odom_delta = math.hypot(dx, dy)
            range_delta = r_center - self.last_front_range

        # update history
        self.last_x = x
        self.last_y = y
        self.last_front_range = r_center

        msg = (
            f"[CHECK] odom=({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f}° | "
            f"line={math.degrees(line_angle):.1f}°, d={d:.3f}m | "
            f"front=[{r_left:.3f}, {r_center:.3f}, {r_right:.3f}]"
        )

        if odom_delta is not None and range_delta is not None:
            msg += f" | Δodom={odom_delta:.3f} m, Δfront={range_delta:.3f} m"

        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
