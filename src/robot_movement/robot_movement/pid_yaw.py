#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

import tf2_ros
from tf_transformations import euler_from_quaternion


class YawPIDController(Node):
    """
    Simple PID controller that aligns robot to a target yaw received via /target_yaw.

    Minimal, navigation-safe change vs. your original:
      - Require yaw error to be within alignment_threshold continuously for stable_time_sec
        (default 0.5s) before publishing /yaw_aligned=True and stopping.

    Everything else is kept the same on purpose.
    """

    def __init__(self):
        super().__init__('yaw_pid_controller')

        # PID parameters
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.86)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('alignment_threshold', 4.0)  # degrees

        # NEW: stable time inside threshold before declaring success
        self.declare_parameter('stable_time_sec', 0.5)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.alignment_threshold = math.radians(float(self.get_parameter('alignment_threshold').value))
        self.stable_time_sec = float(self.get_parameter('stable_time_sec').value)

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Target yaw (radians)
        self.target_yaw = None
        self.is_active = False
        self.alignment_complete = False

        # NEW: time when we first entered the threshold band
        self._in_thresh_since = None  # rclpy.time.Time or None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frame names
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Subscribers
        self.create_subscription(
            Float32,
            '/target_yaw',  # Topic to receive target yaw (in radians)
            self.target_yaw_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_aligned_pub = self.create_publisher(Bool, '/yaw_aligned', 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info("Yaw PID Controller started")
        self.get_logger().info("Waiting for target yaw on /target_yaw topic...")

    def get_current_yaw(self):
        """Get current robot yaw from TF."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.5)
            )

            q = trans.transform.rotation
            quat = np.array([q.x, q.y, q.z, q.w])

            roll, pitch, yaw = euler_from_quaternion(quat)
            return yaw

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF error: {e}")
            return None  # IMPORTANT: don't fake yaw=0.0 on TF failure

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def target_yaw_callback(self, msg):
        """Callback for receiving target yaw."""
        self.target_yaw = float(msg.data)
        self.is_active = True
        self.alignment_complete = False

        # NEW: reset stability timer
        self._in_thresh_since = None

        # Publish NOT aligned immediately
        self.yaw_aligned_pub.publish(Bool(data=False))

        # Reset PID
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        self.get_logger().info(
            f"Received target yaw: {math.degrees(self.target_yaw):.1f}°"
        )

    def compute_pid(self, current_yaw, current_time):
        """Compute PID output for angular velocity."""
        # Calculate time delta
        if self.prev_time is None:
            dt = 0.1  # Default
        else:
            dt_ns = current_time.nanoseconds - self.prev_time.nanoseconds
            dt = dt_ns * 1e-9
            if dt <= 0:
                dt = 0.1

        # Calculate error (normalized)
        error = self.normalize_angle(self.target_yaw - current_yaw)

        # PID terms
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # PID output
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)

        # Limit output
        output = max(-self.max_angular_speed, min(self.max_angular_speed, output))

        # Update state
        self.prev_error = error
        self.prev_time = current_time

        return output, error

    def control_loop(self):
        """Main control loop."""
        # If no target or alignment already complete, do nothing
        if self.target_yaw is None or not self.is_active or self.alignment_complete:
            return

        # Get current yaw
        current_yaw = self.get_current_yaw()
        if current_yaw is None:
            return  # TF not ready, skip this cycle safely

        current_time = self.get_clock().now()

        # Compute PID
        angular_vel, error = self.compute_pid(current_yaw, current_time)

        # ----------------------------
        # NEW: stable-time aligned logic
        # ----------------------------
        if abs(error) < self.alignment_threshold:
            # start or continue timing inside the band
            if self._in_thresh_since is None:
                self._in_thresh_since = current_time

            dt_in = (current_time.nanoseconds - self._in_thresh_since.nanoseconds) * 1e-9

            if dt_in >= self.stable_time_sec:
                if not self.alignment_complete:
                    self.get_logger().info(
                        f"Alignment complete! Error: {math.degrees(error):.1f}° "
                        f"(stable {dt_in:.2f}s)"
                    )
                    self.alignment_complete = True

                    # Notify BT that alignment succeeded
                    self.yaw_aligned_pub.publish(Bool(data=True))

                # Stop the robot
                self.cmd_vel_pub.publish(Twist())
                return
        else:
            # left the band -> reset timer
            self._in_thresh_since = None

        # ----------------------------
        # Normal control output
        # ----------------------------
        twist = Twist()
        twist.angular.z = float(angular_vel)
        self.cmd_vel_pub.publish(twist)

        # Log current status (debug)
        self.get_logger().debug(
            f"Error: {math.degrees(error):.1f}°, Angular vel: {angular_vel:.3f}"
        )

    def reset(self):
        """Reset the controller."""
        self.target_yaw = None
        self.is_active = False
        self.alignment_complete = False
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self._in_thresh_since = None

        # Stop robot
        self.cmd_vel_pub.publish(Twist())
        self.yaw_aligned_pub.publish(Bool(data=False))

        self.get_logger().info("Controller reset")


def main(args=None):
    rclpy.init(args=args)
    node = YawPIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.reset()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
