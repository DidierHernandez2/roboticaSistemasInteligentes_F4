#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan

import tf2_ros
from tf_transformations import euler_from_quaternion


# === Same constants as original ===
LIN_ACC = 0.0015
LIN_DES = -0.0005

MAX_LIN_SPEED_HIGH = 0.45
MAX_LIN_SPEED_MID = 0.15

ANG_ACC_LOW = 0.001
ANG_ACC_MID = 0.003
ANG_ACC_HIGH = 0.006

MAX_ANG_SPEED_LOW = 0.2
MAX_ANG_SPEED_MID = 0.4
MAX_ANG_SPEED_HIGH = 0.6

FORCE_THRESHOLD_LOW = np.pi/10
FORCE_THRESHOLD_MID = np.pi/2
FORCE_THRESHOLD_HIGH = np.pi*(4/5)


class VirtualAttractorNode(Node):
    def __init__(self):
        super().__init__('virtual_attractor_node')

        # HARD-CODED robot settings (change here if needed)
        self.map_frame = "map"
        self.base_frame = "base_link"     # Robotec likely uses base_link
        self.laser_topic = "/scan"        # Robotec LIDAR topic
        self.cmd_vel_topic = "/cmd_vel"  # Output to your mux

        # Internal state
        self.xcl = 0.0
        self.ycl = 0.0
        self.Fx_rep = 0.0
        self.Fy_rep = 0.0
        self.laser_degs = None
        self.current_speed = Twist()
        self.enable_repulsion = True

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(LaserScan, self.laser_topic, self.read_sensor_cb, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.read_point_cb, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 25 Hz control loop
        self.timer = self.create_timer(1.0/25.0, self.control_loop)

        self.get_logger().info("Virtual attractor node started.")

    # === Helpers ===
    @staticmethod
    def tf_2_np_array(t):
        pose = np.array([t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z])
        quat = np.array([t.transform.rotation.x,
                         t.transform.rotation.y,
                         t.transform.rotation.z,
                         t.transform.rotation.w])
        return pose, quat

    def get_robot_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            pose, quat = self.tf_2_np_array(trans)
            roll, pitch, yaw = euler_from_quaternion(quat)
            return pose[0], pose[1], yaw
        except:
            return 0.0, 0.0, 0.0

    # === Force calculations ===
    def calculate_force(self):
        if self.enable_repulsion:
            Fth_rep = math.atan2(self.Fy_rep, self.Fx_rep) + math.pi
            Fmag_rep = math.hypot(self.Fx_rep, self.Fy_rep)
        else:
            Fth_rep = 0.0
            Fmag_rep = 0.0

        x, y, th = self.get_robot_current_pose()

        xy = np.array([x, y])
        xycl = np.array([self.xcl, self.ycl])
        d = np.linalg.norm(xy - xycl)
        if d < 1e-6: d = 1e-6

        Fx_atr = -(x - self.xcl) / d
        Fy_atr = -(y - self.ycl) / d
        Fth_atr = math.atan2(Fy_atr, Fx_atr) - th
        Fmag_atr = math.hypot(Fx_atr, Fy_atr)

        Fx_tot = Fmag_rep * math.cos(Fth_rep) * 0.0025 + Fmag_atr * math.cos(Fth_atr)
        Fy_tot = Fmag_rep * math.sin(Fth_rep) * 0.0025 + Fmag_atr * math.sin(Fth_atr)
        Fth_tot = math.atan2(Fy_tot, Fx_tot)

        return Fx_tot, Fy_tot, Fth_tot, d

    def speed_behavior(self, cur, Fx, Fy, Fth, d):
        out = Twist()

        if abs(Fth) < FORCE_THRESHOLD_LOW:
            out.linear.x = min(cur.linear.x + LIN_ACC, MAX_LIN_SPEED_HIGH)
            out.angular.z = 0.0

        elif abs(Fth) < FORCE_THRESHOLD_MID:
            out.linear.x = max(cur.linear.x + LIN_DES, MAX_LIN_SPEED_MID)
            out.angular.z = max(
                cur.angular.z + ANG_ACC_LOW * math.copysign(1, Fth),
                MAX_ANG_SPEED_LOW * math.copysign(1, Fth)
            )

        elif abs(Fth) < FORCE_THRESHOLD_HIGH:
            out.linear.x = 0.0
            out.angular.z = max(
                cur.angular.z + ANG_ACC_MID * math.copysign(1, Fth),
                MAX_ANG_SPEED_MID * math.copysign(1, Fth)
            )

        else:
            out.linear.x = 0.0
            out.angular.z = max(
                cur.angular.z + ANG_ACC_HIGH * math.copysign(1, Fth),
                MAX_ANG_SPEED_HIGH * math.copysign(1, Fth)
            )

        return out

    def final_turn(self, cur, Fth):
        out = Twist()
        out.angular.z = max(
            cur.angular.z + ANG_ACC_MID * math.copysign(1, Fth),
            MAX_ANG_SPEED_LOW * math.copysign(1, Fth)
        )
        return out

    # === Callbacks ===
    def read_point_cb(self, msg):
        self.xcl = msg.point.x
        self.ycl = msg.point.y
        self.get_logger().info(f"New goal: ({self.xcl:.2f}, {self.ycl:.2f})")

    def read_sensor_cb(self, msg):
        arr = np.asarray(msg.ranges, dtype=float)
        arr[np.isinf(arr)] = 13.5

        if self.laser_degs is None:
            self.laser_degs = np.linspace(msg.angle_min, msg.angle_max, len(arr))

        self.Fx_rep = 0.0
        self.Fy_rep = 0.0

        for r, ang in zip(arr, self.laser_degs):
            inv = (1.0/r)**2
            self.Fx_rep += inv * math.cos(ang)
            self.Fy_rep += inv * math.sin(ang)

    # === Control loop ===
    def control_loop(self):
        # No goal yet
        if self.xcl == 0.0 and self.ycl == 0.0:
            return
        if self.laser_degs is None:
            return

        Fx, Fy, Fth, d = self.calculate_force()

        if d < 1.0:  # hard-coded like original
            self.current_speed.linear.x = 0.0
            self.current_speed.linear.y = 0.0
            self.enable_repulsion = False

            if abs(Fth) > 0.1:
                self.current_speed = self.final_turn(self.current_speed, Fth)
            else:
                self.xcl = 0.0
                self.ycl = 0.0
                self.current_speed = Twist()
                self.get_logger().info("Navigation successful!")

        else:
            self.current_speed = self.speed_behavior(self.current_speed, Fx, Fy, Fth, d)
            self.enable_repulsion = True

        self.cmd_vel_pub.publish(self.current_speed)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualAttractorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
