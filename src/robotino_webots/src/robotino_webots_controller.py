#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState, Image, PointCloud2 ,  PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
import math
import numpy as np
from controller import Robot

# === CAMERA / CV BRIDGE SUPPORT ===
try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class RobotinoWebotsController(Node):

    def __init__(self):
        super().__init__('robotino_webots_controller')

        # --- Parameters ---
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, False)
        ])

        # --- ROS interfaces ---
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 50)
        self.laser_publisher = self.create_publisher(LaserScan, 'scan', 50)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # --- Webots robot ---
        self.robot = Robot()
        self.timestep = 25  # ms

        # --- Wheels ---
        self.wheels = []
        self.wheel_encoders = []

        for i in range(3):
            wheel = self.robot.getDevice(f'wheel{i}_joint')
            wheel.setPosition(float('inf'))
            self.wheels.append(wheel)

            encoder = self.robot.getDevice(f'wheel{i}_joint_sensor')
            encoder.enable(self.timestep)
            self.wheel_encoders.append(encoder)

        self.WHEEL_RADIUS = 0.038
        self.DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826
        self.base_apply_speeds(0.0, 0.0, 0.0)

        # --- Lidar ---
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)

        # --- Camera / Kinect ---
        self.bridge = CvBridge() if CvBridge else None

        self.camera = None
        self.camera_publisher = None
        try:
            self.camera = self.robot.getDevice('camera')
            self.camera.enable(self.timestep)
            if self.bridge:
                self.camera_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        except Exception:
            pass

        self.kinect_rgb = None
        self.kinect_depth = None
        self.kinect_rgb_pub = None
        self.kinect_depth_pub = None
        self.kinect_points_pub = None
        try:
            self.kinect_rgb = self.robot.getDevice('kinect_rgb')
            self.kinect_rgb.enable(self.timestep)
            if self.bridge:
                self.kinect_rgb_pub = self.create_publisher(Image, 'kinect_sim/rgb/image_raw', 10)
            self.kinect_points_pub = self.create_publisher(PointCloud2, '/kinect_sim/points', 10)
        except Exception:
            pass

        try:
            self.kinect_depth = self.robot.getDevice('kinect_depth')
            self.kinect_depth.enable(self.timestep)
            if self.bridge:
                self.kinect_depth_pub = self.create_publisher(Image, 'kinect_sim/depth/image_raw', 10)
        except Exception:
            pass

        # --- Laser parameters ---
        self.scan_angle = 240.0 * np.pi / 180.0
        self.angle_increment = self.scan_angle / 666

        # --- Odometry state (RESTORED) ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # wheel baseline + time baseline (Webots time)
        self.last_wheel_positions = [0.0, 0.0, 0.0]
        self.wheel_encoders_initialized = False
        self.last_time = float(self.robot.getTime())

        # ==========================================================
        # STATIC TFs (PUBLISHED ONCE, LATCHED)
        # ==========================================================
        static_tfs = []

        # base_footprint -> laser_frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        static_tfs.append(t)

        # base_footprint -> camera_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.25
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        static_tfs.append(t)

        # base_footprint -> kinect_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'kinect_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.60
        t.transform.rotation.x = -0.5
        t.transform.rotation.y =  0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w =  0.5
        static_tfs.append(t)

        self.static_tf_broadcaster.sendTransform(static_tfs)

        self.get_logger().info('Robotino Webots controller initialized')

    # =============================================================

    def base_apply_speeds(self, vx, vy, omega):
        vx /= self.WHEEL_RADIUS
        vy /= self.WHEEL_RADIUS
        omega *= self.DISTANCE_WHEEL_TO_ROBOT_CENTRE / self.WHEEL_RADIUS
        self.wheels[0].setVelocity(vy - omega)
        self.wheels[1].setVelocity(-math.sqrt(0.75) * vx - 0.5 * vy - omega)
        self.wheels[2].setVelocity(math.sqrt(0.75) * vx - 0.5 * vy - omega)

    def cmd_vel_callback(self, msg):
        self.base_apply_speeds(msg.linear.x, msg.linear.y, msg.angular.z)

    # =============================================================
    # ODOMETRY (RESTORED: your tuned version)
    # =============================================================
    def update_odometry(self, current_webots_time: float):
        if math.isnan(self.x) or math.isnan(self.y) or math.isnan(self.th):
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0

        try:
            wheel_positions = [e.getValue() for e in self.wheel_encoders]

            if not self.wheel_encoders_initialized:
                self.last_wheel_positions = wheel_positions
                self.wheel_encoders_initialized = True
                self.last_time = current_webots_time
                return

            delta_wheel0 = wheel_positions[0] - self.last_wheel_positions[0]
            delta_wheel1 = wheel_positions[1] - self.last_wheel_positions[1]
            delta_wheel2 = wheel_positions[2] - self.last_wheel_positions[2]
            self.last_wheel_positions = wheel_positions

            # YOUR tuned gains
            delta_x = self.WHEEL_RADIUS * 1.099 * (0.0000 * delta_wheel0 - 0.8660 * delta_wheel1 + 0.8660 * delta_wheel2)
            delta_y = self.WHEEL_RADIUS * 1.099 * (1.0000 * delta_wheel0 - 0.5000 * delta_wheel1 - 0.5000 * delta_wheel2)
            delta_th = self.WHEEL_RADIUS * -1.617 * (delta_wheel0 + delta_wheel1 + delta_wheel2) / (3.0 * self.DISTANCE_WHEEL_TO_ROBOT_CENTRE)

            dt = current_webots_time - self.last_time
            if dt > 0.0:
                self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
                self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
                self.th += delta_th
                self.th = math.atan2(math.sin(self.th), math.cos(self.th))

            self.last_time = current_webots_time

        except Exception as e:
            self.get_logger().warn(f'Odometry update error: {e}')

    # =============================================================
    from sensor_msgs.msg import PointCloud2, PointField
    import struct

    def make_kinect_pointcloud_msg(self, depth_np, stamp):
        height, width = depth_np.shape

        hfov = self.kinect_depth.getFov()
        vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * (height / width))

        fx = width / (2.0 * math.tan(hfov / 2.0))
        fy = height / (2.0 * math.tan(vfov / 2.0))
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0

        u, v = np.meshgrid(np.arange(width), np.arange(height))

        z = depth_np.astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # keep full image shape, do NOT flatten
        xyz = np.stack((x, y, z), axis=-1).astype(np.float32)

        # invalid pixels -> NaN, but keep them in place
        invalid = ~np.isfinite(z) | (z <= 0.0)
        xyz[invalid] = np.nan

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'kinect_link'

        msg.height = height
        msg.width = width

        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * width
        msg.is_dense = False
        msg.data = xyz.tobytes()

        return msg
    def run(self):
        # ensure odom starts clean each run
        self.wheel_encoders_initialized = False
        self.last_time = float(self.robot.getTime())

        while rclpy.ok() and self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0)

            now = self.get_clock().now().to_msg()

            self.update_odometry(float(self.robot.getTime()))

            # --- LaserScan ---
            ranges = self.lidar.getRangeImage()
            if ranges:
                scan = LaserScan()
                scan.header.stamp = now
                scan.header.frame_id = 'laser_frame'
                scan.angle_min = -self.scan_angle / 2
                scan.angle_max = self.scan_angle / 2
                scan.angle_increment = self.angle_increment
                scan.range_min = 0.05
                scan.range_max = 10.0
                scan.ranges = list(reversed(ranges))
                self.laser_publisher.publish(scan)

            # --- Odometry ---
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
            self.odom_publisher.publish(odom)

            # --- TF odom -> base_footprint (ONLY dynamic TF) ---
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.th / 2.0)
            t.transform.rotation.w = math.cos(self.th / 2.0)
            self.tf_broadcaster.sendTransform(t)

            # --- Joint states ---
            js = JointState()
            js.header.stamp = now
            js.name = ['wheel0_joint', 'wheel1_joint', 'wheel2_joint']
            js.position = [e.getValue() for e in self.wheel_encoders]
            self.joint_publisher.publish(js)

                        # --- Camera publish (Webots -> ROS Image) ---
            if self.bridge and self.camera_publisher and self.camera:
                try:
                    img = self.camera.getImage()
                    if img is not None:
                        width = self.camera.getWidth()
                        height = self.camera.getHeight()
                        # Webots camera returns BGRA bytes
                        np_img = np.frombuffer(img, dtype=np.uint8).reshape((height, width, 4))
                        bgr = np_img[:, :, :3]  # drop alpha

                        msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                        msg.header.stamp = now
                        msg.header.frame_id = 'camera_link'
                        self.camera_publisher.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f'Camera publish error: {e}')

            # --- Kinect RGB publish ---
            if self.bridge and self.kinect_rgb_pub and self.kinect_rgb:
                try:
                    img = self.kinect_rgb.getImage()
                    if img is not None:
                        width = self.kinect_rgb.getWidth()
                        height = self.kinect_rgb.getHeight()
                        np_img = np.frombuffer(img, dtype=np.uint8).reshape((height, width, 4))
                        bgr = np_img[:, :, :3]

                        msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                        msg.header.stamp = now
                        msg.header.frame_id = 'kinect_link'
                        self.kinect_rgb_pub.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f'Kinect RGB publish error: {e}')

            # --- Kinect Depth publish (as 32FC1) ---
            if self.bridge and self.kinect_depth_pub and self.kinect_depth:
                try:
                    depth = self.kinect_depth.getRangeImage()
                    if depth is not None:
                        width = self.kinect_depth.getWidth()
                        height = self.kinect_depth.getHeight()
                        depth_np = np.array(depth, dtype=np.float32).reshape((height, width))

                        msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
                        msg.header.stamp = now
                        msg.header.frame_id = 'kinect_link'
                        self.kinect_depth_pub.publish(msg)
                        if self.kinect_points_pub:
                            cloud_msg = self.make_kinect_pointcloud_msg(depth_np, now)
                            self.kinect_points_pub.publish(cloud_msg)
                except Exception as e:
                    self.get_logger().warn(f'Kinect depth publish error: {e}')



def main():
    rclpy.init()
    node = RobotinoWebotsController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
