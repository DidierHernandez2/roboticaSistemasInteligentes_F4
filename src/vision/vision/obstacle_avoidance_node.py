#!/usr/bin/env python3
"""
obstacle_avoidance_node.py

Velocity filter that sits between any navigation node and the robot.

  [yolo_person_node] --/cmd_vel_desired--> [THIS NODE] --/cmd_vel--> robot
                                                 ↑
                                          /scan  +  /depth

Repulsion sources:
  1. LIDAR  (/scan)          – legs, walls, floor-level obstacles
  2. Depth band              – table-top height obstacles LIDAR misses

The node reads the desired velocity, applies an obstacle-repulsion correction
(reduce forward speed + steer away), and publishes the safe velocity.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
REP_MIN_MAG      = 50.0    # combined repulsion magnitude below which no correction is applied
REP_LIN_GAIN     = 0.003   # how much forward speed is reduced per unit repulsion
REP_ANG_GAIN     = 0.004   # how much angular correction is added per unit repulsion
MAX_ANG_CORR     = 0.5     # maximum angular correction (rad/s)

DEPTH_MIN_DIST   = 0.3     # ignore depth closer than this (m) – sensor noise floor
DEPTH_REP_WEIGHT = 0.15    # scale depth repulsion relative to LIDAR
                            # (depth band has ~10x more rays than LIDAR in same FOV)


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Parameters
        self.declare_parameter('laser_topic',    '/scan')
        self.declare_parameter('depth_topic',    '/kinect_sim/depth/image_raw')
        self.declare_parameter('cmd_vel_in',     '/cmd_vel_desired')
        self.declare_parameter('cmd_vel_out',    '/cmd_vel')
        self.declare_parameter('depth_row_min',    150)   # ← adjust to match table height
        self.declare_parameter('depth_row_max',    330)  # ← on your specific Kinect mount
        self.declare_parameter('lidar_max_range',  1.5)    # LIDAR: ignore obstacles farther than this (m)
        self.declare_parameter('depth_max_range',  1.5)    # Depth: ignore obstacles farther than this (m)
        self.declare_parameter('rep_min_mag',      50.0)   # repulsion threshold – raise to react less
        self.declare_parameter('rep_lin_gain',     0.003)  # forward speed reduction per unit repulsion
        self.declare_parameter('rep_ang_gain',     0.004)  # angular correction per unit repulsion
        self.declare_parameter('max_ang_corr',     0.5)    # max angular correction (rad/s)

        laser_topic  = self.get_parameter('laser_topic').get_parameter_value().string_value
        depth_topic  = self.get_parameter('depth_topic').get_parameter_value().string_value
        cmd_vel_in   = self.get_parameter('cmd_vel_in').get_parameter_value().string_value
        cmd_vel_out  = self.get_parameter('cmd_vel_out').get_parameter_value().string_value
        self.row_min         = self.get_parameter('depth_row_min').get_parameter_value().integer_value
        self.row_max         = self.get_parameter('depth_row_max').get_parameter_value().integer_value
        self.lidar_max_range = self.get_parameter('lidar_max_range').get_parameter_value().double_value
        self.depth_max_range = self.get_parameter('depth_max_range').get_parameter_value().double_value
        self.rep_min_mag     = self.get_parameter('rep_min_mag').get_parameter_value().double_value
        self.rep_lin_gain    = self.get_parameter('rep_lin_gain').get_parameter_value().double_value
        self.rep_ang_gain    = self.get_parameter('rep_ang_gain').get_parameter_value().double_value
        self.max_ang_corr    = self.get_parameter('max_ang_corr').get_parameter_value().double_value

        # Kinect horizontal intrinsics (used to convert column → angle in robot frame)
        self.fx = 525.0
        self.cx = 319.5

        self.bridge     = CvBridge()
        self.laser_degs = None

        # Repulsion force components (updated by sensor callbacks)
        self.Fx_lidar = 0.0
        self.Fy_lidar = 0.0
        self.Fx_depth = 0.0
        self.Fy_depth = 0.0

        # Latest desired velocity from navigation node
        self.desired_vel = Twist()

        # Subscriptions
        self.create_subscription(LaserScan, laser_topic, self.laser_callback, 10)
        self.create_subscription(Image,     depth_topic, self.depth_callback,  10)
        self.create_subscription(Twist,     cmd_vel_in,  self.cmd_vel_callback, 10)

        # Output publisher
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_out, 10)

        # 25 Hz output loop (matches yolo_person_node control rate)
        self.create_timer(1.0 / 25.0, self.output_loop)

        self.get_logger().info(
            f'[obstacle_avoidance] Ready | '
            f'Laser: {laser_topic} (max {self.lidar_max_range} m) | '
            f'Depth: {depth_topic} (max {self.depth_max_range} m, rows [{self.row_min},{self.row_max}]) | '
            f'In: {cmd_vel_in} → Out: {cmd_vel_out}'
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        self.desired_vel = msg

    def laser_callback(self, msg: LaserScan):
        """Compute LIDAR repulsion force (same method as pot_fields.py)."""
        arr = np.asarray(msg.ranges, dtype=float)
        arr[np.isinf(arr)] = 13.5

        if self.laser_degs is None:
            self.laser_degs = np.linspace(msg.angle_min, msg.angle_max, len(arr))

        fx, fy = 0.0, 0.0
        for r, ang in zip(arr, self.laser_degs):
            if r > self.lidar_max_range:   # ignore far obstacles (walls, etc.)
                continue
            inv  = (1.0 / r) ** 2
            fx  += inv * math.cos(ang)
            fy  += inv * math.sin(ang)
        self.Fx_lidar, self.Fy_lidar = fx, fy

    def depth_callback(self, msg: Image):
        """
        Compute depth-band repulsion force.

        Takes a horizontal slice of the depth image (rows depth_row_min..depth_row_max)
        that corresponds to table-top height.  For each column the minimum depth in that
        band represents the closest mid-height obstacle in that direction.
        """
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if raw.dtype == np.uint16:
            depth = raw.astype(np.float32) / 1000.0   # mm → m
        else:
            depth = raw.astype(np.float32)

        h, w  = depth.shape[:2]
        r_min = max(0, self.row_min)
        r_max = min(h, self.row_max)

        band = depth[r_min:r_max, :]   # (band_rows, width)

        # Keep only valid range; set everything else to inf so min() ignores it
        with np.errstate(invalid='ignore'):
            valid = (band > DEPTH_MIN_DIST) & (band < self.depth_max_range) & np.isfinite(band)
            band  = np.where(valid, band, np.inf)

        col_min = np.min(band, axis=0)   # closest obstacle per column, shape (width,)

        fx, fy = 0.0, 0.0
        for col, d in enumerate(col_min):
            if not np.isfinite(d):
                continue
            # Angle in robot frame: forward=0, left>0, right<0
            # (left side of image = col < cx = robot's right ... after kinect→base rotation)
            ang  = math.atan2(self.cx - col, self.fx)
            inv  = (1.0 / d) ** 2 * DEPTH_REP_WEIGHT
            fx  += inv * math.cos(ang)
            fy  += inv * math.sin(ang)
        self.Fx_depth, self.Fy_depth = fx, fy

    # -----------------------------------------------------------------------
    # Velocity filter
    # -----------------------------------------------------------------------
    def _filter_velocity(self, desired: Twist) -> Twist:
        """
        Apply obstacle repulsion correction to the desired velocity.

        Logic:
          - If robot is not moving forward (linear.x == 0) → pass through unchanged.
            This means SEARCHING (pure spin) and MEASURING_DISTANCE (stopped) are
            never affected by obstacle avoidance.
          - If combined repulsion is weak  → pass through unchanged
          - If obstacle is ahead           → reduce linear speed
          - Always (when moving forward)   → add angular correction to steer away
        """
        # Only filter when the robot is actually trying to move forward
        if desired.linear.x <= 0.0:
            return desired

        Fx_rep = self.Fx_lidar + self.Fx_depth
        Fy_rep = self.Fy_lidar + self.Fy_depth
        Fmag   = math.hypot(Fx_rep, Fy_rep)

        if Fmag < self.rep_min_mag:
            return desired

        # Direction the robot should move to get away from obstacles [-π, π]
        Fth_away = math.atan2(Fy_rep, Fx_rep) + math.pi
        Fth_away = (Fth_away + math.pi) % (2.0 * math.pi) - math.pi

        out = Twist()

        # 1. Reduce forward speed when obstacle is ahead
        #    cos(Fth_away) < 0  →  repulsion pushes backward  →  obstacle is ahead
        cos_away = math.cos(Fth_away)
        if cos_away < 0.0:
            reduction    = (-cos_away) * Fmag * self.rep_lin_gain
            out.linear.x = max(0.0, desired.linear.x * (1.0 - reduction))
        else:
            out.linear.x = desired.linear.x

        # 2. Steer away: add angular correction toward repulsion direction
        ang_corr      = math.sin(Fth_away) * Fmag * self.rep_ang_gain
        ang_corr      = max(-self.max_ang_corr, min(self.max_ang_corr, ang_corr))
        out.angular.z = desired.angular.z + ang_corr

        return out

    # -----------------------------------------------------------------------
    # 25 Hz output
    # -----------------------------------------------------------------------
    def output_loop(self):
        self.cmd_vel_pub.publish(self._filter_velocity(self.desired_vel))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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
