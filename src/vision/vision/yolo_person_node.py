#!/usr/bin/env python3
"""
yolo_person_node.py

State machine:
  SEARCHING                    – robot spins until person detected
  PERSON_FOUND                 – robot stops, waits for depth data
  MEASURING_DISTANCE           – computes 3-D person position via depth × YOLO mask,
                                  transforms to map frame and sets navigation goal
  MOVING_AND_OBSTACLEDETECTION – publishes attraction-only velocity to /cmd_vel_desired;
                                  obstacle_avoidance_node filters it before reaching /cmd_vel

Flow:
  SEARCHING → PERSON_FOUND → MEASURING_DISTANCE → MOVING_AND_OBSTACLEDETECTION
      ↑                                                        |
      └────────────────── person lost / arrived ───────────────┘
"""
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from enum import Enum
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs   # noqa: F401  – required for tf_buffer.transform(PointStamped)
from tf_transformations import euler_from_quaternion


# ---------------------------------------------------------------------------
# Attraction speed-behavior constants  (same as pot_fields.py)
# ---------------------------------------------------------------------------
LIN_ACC  = 0.0015
LIN_DES  = -0.0005

MAX_LIN_SPEED_HIGH = 0.20   # was 0.45 – slower so YOLO can track while moving
MAX_LIN_SPEED_MID  = 0.08   # was 0.15

ANG_ACC_LOW  = 0.001
ANG_ACC_MID  = 0.003
ANG_ACC_HIGH = 0.006

MAX_ANG_SPEED_LOW  = 0.15   # was 0.2
MAX_ANG_SPEED_MID  = 0.25   # was 0.4
MAX_ANG_SPEED_HIGH = 0.35   # was 0.6

FORCE_THRESHOLD_LOW  = math.pi / 10        # ~18°
FORCE_THRESHOLD_MID  = math.pi / 2         # 90°
FORCE_THRESHOLD_HIGH = math.pi * (4 / 5)   # 144°

ARRIVAL_DIST = 1.0   # metres – stop and re-measure when this close


# ---------------------------------------------------------------------------
class State(Enum):
    MAPPING                      = 'MAPPING'
    SEARCHING                    = 'SEARCHING'
    PERSON_FOUND                 = 'PERSON_FOUND'
    MEASURING_DISTANCE           = 'MEASURING_DISTANCE'
    PATH_PLANNING                = 'PATH_PLANNING'
    MOVING_AND_OBSTACLEDETECTION = 'MOVING_AND_OBSTACLEDETECTION'
    WANDERING                    = 'WANDERING'
    ARRIVED                      = 'ARRIVED'


# ---------------------------------------------------------------------------
class YoloPersonNode(Node):
    def __init__(self):
        super().__init__('yolo_person_node')

        self.declare_parameter('image_topic',    '/kinect_sim/rgb/image_raw')
        self.declare_parameter('depth_topic',    '/kinect_sim/depth/image_raw')

        self.declare_parameter('model_path',     'yolo11n-seg.pt')
        self.declare_parameter('search_ang_vel', 0.1)   # rad/s while spinning in SEARCHING

        image_topic          = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic          = self.get_parameter('depth_topic').get_parameter_value().string_value
        model_path           = self.get_parameter('model_path').get_parameter_value().string_value
        self.search_ang_vel  = self.get_parameter('search_ang_vel').get_parameter_value().double_value

        # Kinect intrinsics
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 319.5, 239.5

        # TF frames
        self.map_frame  = 'map'
        self.base_frame = 'base_link'

        # State
        self.state         = State.MAPPING
        self.current_speed = Twist()

        # MAPPING state: spin 360° while slam_toolbox builds the map
        _mapping_ang_vel          = self.search_ang_vel   # reuse same spin speed
        self._mapping_ticks_total = math.ceil(2 * math.pi / _mapping_ang_vel / (1.0 / 25.0))
        self._mapping_ticks_done  = 0
        self._mapping_log_counter = 0

        # Sensor data
        self.latest_depth = None   # float32 depth image, metres

        # Navigation goal in map frame (x, y)
        self.person_goal  = None

        # A* path following
        self.path_waypoints:    list  = []    # [(x, y), ...] waypoints en frame map
        self.path_index:        int   = 0     # waypoint actual
        self._path_future             = None  # future del servicio compute_path
        self._path_request_sent: bool = False

        # Periodic replanning during MOVING (each REPLAN_INTERVAL ticks @ 25 Hz)
        self._REPLAN_INTERVAL = 5 * 25        # 5 segundos
        self._replan_counter  = 0

        # Stuck detection & noise perturbation
        self._last_check_pos    = None   # (x, y) sampled every second
        self._stuck_ticks       = 0      # consecutive seconds without movement
        self._stuck_threshold   = 3      # seconds before injecting noise
        self._wander_threshold  = 7      # seconds stuck before triggering WANDERING
        self._noise_magnitude   = 0.0    # current noise std (grows while stuck)
        self._pos_check_counter = 0
        self._trigger_wander    = False  # flag set by _check_stuck
        self._stuck_repulsors   = []     # list of (x, y) map positions where robot got stuck

        # WANDERING state (back-up escape)
        WANDER_DURATION_S       = 4      # seconds to reverse before retrying
        self._wander_ticks_left = 0
        self._wander_duration   = WANDER_DURATION_S * 25
        self._wander_ang_sign   = 1      # +1 or -1, chosen when entering WANDERING

        # Throttle logs (~1 s = 25 ticks)
        self._move_log_counter   = 0
        self._search_log_counter = 0
        self._image_count        = 0   # frames received, to detect missing topic

        # ROS
        self.bridge = CvBridge()
        self.model  = YOLO(model_path)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # NOTE: publishes to /cmd_vel_desired – obstacle_avoidance_node filters it
        self.cmd_vel_pub  = self.create_publisher(Twist, '/cmd_vel_desired', 10)
        self.state_pub    = self.create_publisher(String, '/robot_state', 10)
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        _path_qos = QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub      = self.create_publisher(Path, '/planned_path', _path_qos)
        self.goal_pub      = self.create_publisher(PointStamped, '/person_goal', 10)
        self.path_client   = self.create_client(GetPlan, 'compute_path')

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback,  10)

        # 25 Hz control loop
        self.create_timer(1.0 / 25.0, self.control_loop)
        # Publish state periodically so map_trace_node always has the current value
        self.create_timer(1.0, lambda: self.state_pub.publish(String(data=self.state.value)))

        cv2.namedWindow('YOLO Person Detection', cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f'[yolo_person] Started | State: {self.state.value} | '
            f'RGB: {image_topic} | Depth: {depth_topic}'
        )

    # -----------------------------------------------------------------------
    # Sensor callbacks
    # -----------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if raw.dtype == np.uint16:
            self.latest_depth = raw.astype(np.float32) / 1000.0
        else:
            self.latest_depth = raw.astype(np.float32)


    # -----------------------------------------------------------------------
    # RGB / YOLO callback  –  state transitions only
    # -----------------------------------------------------------------------
    def image_callback(self, msg: Image):
        frame   = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)

        names   = results[0].names
        classes = (results[0].boxes.cls.cpu().numpy().astype(int)
                   if results[0].boxes else [])
        person_detected = any(names.get(c) == 'person' for c in classes)

        person_mask = None
        if results[0].masks is not None:
            for i, cls_id in enumerate(classes):
                if names.get(cls_id) == 'person':
                    raw = results[0].masks.data[i].cpu().numpy()
                    person_mask = (raw > 0.5).astype(np.uint8)
                    break

        self._image_count += 1
        self._update_state(person_detected, person_mask)

        annotated = results[0].plot()
        cv2.imshow('YOLO Person Detection', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    # -----------------------------------------------------------------------
    def _publish_state(self):
        self.state_pub.publish(String(data=self.state.value))

    # -----------------------------------------------------------------------
    # State machine  –  transitions only, no cmd_vel publishing here
    # -----------------------------------------------------------------------
    def _update_state(self, person_detected: bool, person_mask):
        if self.state == State.MAPPING:
            pass   # mapping completes via tick counter in control_loop, ignore detections

        elif self.state == State.SEARCHING:
            if person_detected:
                self.state = State.PERSON_FOUND
                self._publish_state()
                self.get_logger().info('[yolo_person] SEARCHING -> PERSON_FOUND')

        elif self.state == State.PERSON_FOUND:
            if not person_detected:
                self.state = State.SEARCHING
                self._publish_state()
                self.get_logger().info('[yolo_person] PERSON_FOUND -> SEARCHING | Person lost')
            elif person_mask is not None and self.latest_depth is not None:
                self.state = State.MEASURING_DISTANCE
                self._publish_state()
                self.get_logger().info('[yolo_person] PERSON_FOUND -> MEASURING_DISTANCE')

        elif self.state == State.MEASURING_DISTANCE:
            if not person_detected:
                self.state = State.SEARCHING
                self._publish_state()
                self.get_logger().info('[yolo_person] MEASURING_DISTANCE -> SEARCHING | Person lost')
            elif person_mask is not None and self.latest_depth is not None:
                goal = self._measure_and_log(person_mask, silent=False)
                if goal is not None:
                    self.person_goal          = goal
                    self.current_speed        = Twist()
                    self._move_log_counter    = 0
                    self._last_check_pos      = None
                    self._stuck_ticks         = 0
                    self._noise_magnitude     = 0.0
                    self._pos_check_counter   = 0
                    self._stuck_repulsors     = []
                    self.path_waypoints       = []
                    self.path_index           = 0
                    self._path_request_sent   = False
                    self._path_future         = None
                    # Publicar goal para potential_field_viz_node
                    pt = PointStamped()
                    pt.header.frame_id  = 'map'
                    pt.header.stamp     = self.get_clock().now().to_msg()
                    pt.point.x, pt.point.y = goal
                    self.goal_pub.publish(pt)
                    self.state = State.PATH_PLANNING
                    self._publish_state()
                    self.get_logger().info(
                        f'[yolo_person] MEASURING_DISTANCE -> PATH_PLANNING | '
                        f'Goal (map): ({goal[0]:.2f}, {goal[1]:.2f})'
                    )

        elif self.state == State.PATH_PLANNING:
            pass   # esperando respuesta de A* — ignorar detecciones

        elif self.state == State.MOVING_AND_OBSTACLEDETECTION:
            pass   # goal is fixed – navigate to saved position, no re-detection needed

        elif self.state == State.WANDERING:
            pass   # ignore YOLO detections while wandering – goal already saved

        elif self.state == State.ARRIVED:
            pass   # terminal state – robot stopped at goal

    # -----------------------------------------------------------------------
    # Depth × mask  →  3-D position  →  map-frame (x, y) goal
    # -----------------------------------------------------------------------
    def _measure_and_log(self, person_mask: np.ndarray, silent: bool = False):
        depth = self.latest_depth.copy()

        h_d, w_d = depth.shape[:2]
        h_m, w_m = person_mask.shape[:2]
        if (h_m, w_m) != (h_d, w_d):
            person_mask = cv2.resize(person_mask, (w_d, h_d),
                                     interpolation=cv2.INTER_NEAREST)

        masked_depth = depth * person_mask.astype(np.float32)
        valid_mask   = (person_mask > 0) & (masked_depth > 0.0) & np.isfinite(masked_depth)
        valid_vals   = masked_depth[valid_mask]

        if valid_vals.size == 0:
            if not silent:
                self.get_logger().warn(
                    '[yolo_person] MEASURING_DISTANCE | No valid depth inside person mask')
            return None

        mean_dist = float(np.mean(valid_vals))
        ys, xs    = np.where(valid_mask)
        cx_px     = float(np.mean(xs))
        cy_px     = float(np.mean(ys))

        # Back-project to kinect_link frame
        X = (cx_px - self.cx) * mean_dist / self.fx
        Y = (cy_px - self.cy) * mean_dist / self.fy
        Z = mean_dist

        if not silent:
            self.get_logger().info(
                f'[yolo_person] MEASURING_DISTANCE | Distance: {mean_dist:.3f} m | '
                f'3D (kinect_link): X={X:.3f} m  Y={Y:.3f} m  Z={Z:.3f} m'
            )

        # Transform to map frame
        try:
            pt = PointStamped()
            pt.header.frame_id = 'kinect_link'
            pt.header.stamp    = rclpy.time.Time().to_msg()   # Time(0) = latest available TF
            pt.point.x, pt.point.y, pt.point.z = X, Y, Z
            pt_map = self.tf_buffer.transform(
                pt, self.map_frame, timeout=Duration(seconds=0.3))
            return (pt_map.point.x, pt_map.point.y)
        except Exception as e:
            self.get_logger().warn(f'[yolo_person] TF kinect_link->map failed: {e}')
            return None

    # -----------------------------------------------------------------------
    # Attraction-only force  (obstacle repulsion lives in obstacle_avoidance_node)
    # -----------------------------------------------------------------------
    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            q = [t.transform.rotation.x, t.transform.rotation.y,
                 t.transform.rotation.z, t.transform.rotation.w]
            _, _, yaw = euler_from_quaternion(q)
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None

    def _check_stuck(self, x: float, y: float):
        """
        Sampled once per second.
        - After _stuck_threshold s: injects gaussian noise on Fth (soft escape).
        - After _wander_threshold s: sets _trigger_wander so the control loop
          transitions to WANDERING (hard escape).
        Resets all counters when the robot moves > 5 cm.
        """
        self._pos_check_counter += 1
        if self._pos_check_counter < 25:
            return
        self._pos_check_counter = 0

        if self._last_check_pos is None:
            self._last_check_pos = (x, y)
            return

        dist_moved = math.hypot(x - self._last_check_pos[0],
                                y - self._last_check_pos[1])
        self._last_check_pos = (x, y)

        if dist_moved < 0.05:
            self._stuck_ticks += 1

            if self._stuck_ticks >= self._wander_threshold:
                # Hard escape: trigger WANDERING + save stuck position as repulsor
                self._trigger_wander  = True
                self._noise_magnitude = 0.0
                self._stuck_repulsors.append((x, y))
                self.get_logger().warn(
                    f'[yolo_person] STUCK {self._stuck_ticks}s → WANDERING | '
                    f'Repulsor added at ({x:.2f}, {y:.2f}) '
                    f'[total: {len(self._stuck_repulsors)}]'
                )
            elif self._stuck_ticks >= self._stuck_threshold:
                # Soft escape: ramp up noise on the attraction angle
                self._noise_magnitude = min(
                    math.pi / 3,
                    (self._stuck_ticks - self._stuck_threshold + 1) * 0.15
                )
                self.get_logger().warn(
                    f'[yolo_person] LOCAL MINIMUM ({self._stuck_ticks}s) | '
                    f'noise={math.degrees(self._noise_magnitude):.1f} deg'
                )
        else:
            if self._stuck_ticks >= self._stuck_threshold:
                self.get_logger().info('[yolo_person] Escaped local minimum')
            self._stuck_ticks     = 0
            self._noise_magnitude = 0.0
            self._trigger_wander  = False

    def _calculate_attraction(self, xcl: float, ycl: float):
        """Returns (Fth_tot, distance_to_goal) or None if TF unavailable."""
        pose = self._get_robot_pose()
        if pose is None:
            return None
        x, y, th = pose

        d = math.hypot(x - xcl, y - ycl)
        if d < 1e-6:
            d = 1e-6

        # Unit attraction vector toward goal (in robot frame angle)
        Fx = math.cos(math.atan2(ycl - y, xcl - x) - th)
        Fy = math.sin(math.atan2(ycl - y, xcl - x) - th)

        # Repulsion from stuck-point memory (map frame, converted to robot frame)
        STUCK_REP_RADIUS = 1.5   # metres – only repels within this radius
        STUCK_REP_WEIGHT = 0.8   # strength relative to attraction unit vector
        for (sx, sy) in self._stuck_repulsors:
            ds = math.hypot(x - sx, y - sy)
            if ds < 1e-3:
                ds = 1e-3
            if ds > STUCK_REP_RADIUS:
                continue
            # Repulsion direction: away from stuck point, in robot frame
            rep_angle = math.atan2(y - sy, x - sx) - th
            mag = STUCK_REP_WEIGHT / (ds ** 2)
            Fx += mag * math.cos(rep_angle)
            Fy += mag * math.sin(rep_angle)

        Fth_tot = math.atan2(Fy, Fx)

        # Soft escape: inject gaussian noise when stuck
        self._check_stuck(x, y)
        if self._noise_magnitude > 0.0:
            Fth_tot += random.gauss(0.0, self._noise_magnitude)

        return Fth_tot, d

    def _speed_behavior(self, cur: Twist, Fth: float) -> Twist:
        out = Twist()
        if abs(Fth) < FORCE_THRESHOLD_LOW:
            out.linear.x  = min(cur.linear.x + LIN_ACC, MAX_LIN_SPEED_HIGH)
            out.angular.z = 0.0
        elif abs(Fth) < FORCE_THRESHOLD_MID:
            out.linear.x  = max(cur.linear.x + LIN_DES, MAX_LIN_SPEED_MID)
            out.angular.z = max(
                cur.angular.z + ANG_ACC_LOW * math.copysign(1, Fth),
                MAX_ANG_SPEED_LOW * math.copysign(1, Fth))
        elif abs(Fth) < FORCE_THRESHOLD_HIGH:
            out.linear.x  = 0.0
            out.angular.z = max(
                cur.angular.z + ANG_ACC_MID * math.copysign(1, Fth),
                MAX_ANG_SPEED_MID * math.copysign(1, Fth))
        else:
            out.linear.x  = 0.0
            out.angular.z = max(
                cur.angular.z + ANG_ACC_HIGH * math.copysign(1, Fth),
                MAX_ANG_SPEED_HIGH * math.copysign(1, Fth))
        return out

    # -----------------------------------------------------------------------
    # 25 Hz control loop  –  ONLY place that publishes cmd_vel_desired
    # -----------------------------------------------------------------------
    def control_loop(self):
        if self.state == State.MAPPING:
            twist = Twist()
            twist.angular.z = self.search_ang_vel
            self.cmd_vel_pub.publish(twist)

            self._mapping_ticks_done  += 1
            self._mapping_log_counter += 1

            if self._mapping_log_counter >= 25:
                self._mapping_log_counter = 0
                pct = self._mapping_ticks_done / self._mapping_ticks_total * 100
                self.get_logger().info(
                    f'[yolo_person] MAPPING | spinning 360° for LIDAR map... '
                    f'{pct:.0f}% ({self._mapping_ticks_done}/{self._mapping_ticks_total} ticks)'
                )

            if self._mapping_ticks_done >= self._mapping_ticks_total:
                self.cmd_vel_pub.publish(Twist())   # stop
                self.state = State.SEARCHING
                self._publish_state()
                self.get_logger().info('[yolo_person] MAPPING complete → SEARCHING')

        elif self.state == State.SEARCHING:
            twist = Twist()
            twist.angular.z = self.search_ang_vel
            self.cmd_vel_pub.publish(twist)

            self._search_log_counter += 1
            if self._search_log_counter >= 25:
                self._search_log_counter = 0
                frames = self._image_count
                self._image_count = 0
                self.get_logger().info(
                    f'[yolo_person] SEARCHING | spinning... | '
                    f'frames/s: {frames} | depth: {"OK" if self.latest_depth is not None else "NO DATA"}'
                )

        elif self.state == State.PATH_PLANNING:
            self.cmd_vel_pub.publish(Twist())   # detenido mientras planifica

            if not self._path_request_sent:
                pose = self._get_robot_pose()
                if pose is None:
                    return
                x, y, _ = pose

                if not self.path_client.service_is_ready():
                    self.get_logger().warn('[yolo_person] PATH_PLANNING | servicio compute_path no disponible aún...')
                    return

                req = GetPlan.Request()
                req.start.header.frame_id      = 'map'
                req.start.pose.position.x      = x
                req.start.pose.position.y      = y
                req.goal.header.frame_id       = 'map'
                req.goal.pose.position.x       = self.person_goal[0]
                req.goal.pose.position.y       = self.person_goal[1]

                self._path_future       = self.path_client.call_async(req)
                self._path_request_sent = True
                self.get_logger().info(
                    f'[yolo_person] PATH_PLANNING | Petición A* enviada | '
                    f'start=({x:.2f},{y:.2f}) goal=({self.person_goal[0]:.2f},{self.person_goal[1]:.2f})'
                )

            elif self._path_future is not None and self._path_future.done():
                result = self._path_future.result()
                if result is not None and len(result.plan.poses) > 0:
                    self.path_waypoints = [
                        (p.pose.position.x, p.pose.position.y)
                        for p in result.plan.poses
                    ]
                    self.path_pub.publish(result.plan)
                    self.get_logger().info(
                        f'[yolo_person] PATH_PLANNING -> MOVING | '
                        f'{len(self.path_waypoints)} waypoints'
                    )
                else:
                    # Sin camino A* — navegación directa como fallback
                    self.path_waypoints = [self.person_goal]
                    self.get_logger().warn(
                        '[yolo_person] PATH_PLANNING | A* sin camino — navegación directa'
                    )

                self.path_index         = 0
                self._path_request_sent = False
                self._path_future       = None
                self._replan_counter    = 0
                self.state = State.MOVING_AND_OBSTACLEDETECTION
                self._publish_state()

        elif self.state in (State.PERSON_FOUND, State.MEASURING_DISTANCE, State.ARRIVED):
            self.cmd_vel_pub.publish(Twist())   # stop

        elif self.state == State.MOVING_AND_OBSTACLEDETECTION:
            if self.person_goal is None:
                return

            # Replan periódico: cada REPLAN_INTERVAL ticks volver a PATH_PLANNING
            self._replan_counter += 1
            if self._replan_counter >= self._REPLAN_INTERVAL:
                self._replan_counter    = 0
                self._path_request_sent = False
                self._path_future       = None
                self.state = State.PATH_PLANNING
                self._publish_state()
                self.get_logger().info(
                    '[yolo_person] MOVING → PATH_PLANNING | Replanning con mapa actualizado'
                )
                return

            # Objetivo actual: waypoint del camino A* o goal directo si no hay path
            is_last_waypoint = (
                not self.path_waypoints or
                self.path_index >= len(self.path_waypoints) - 1
            )
            if self.path_waypoints and self.path_index < len(self.path_waypoints):
                xcl, ycl = self.path_waypoints[self.path_index]
            else:
                xcl, ycl = self.person_goal

            result = self._calculate_attraction(xcl, ycl)
            if result is None:
                return
            Fth, d = result

            self._move_log_counter += 1
            if self._move_log_counter >= 25:
                self._move_log_counter = 0
                n_wp  = len(self.path_waypoints)
                wp_str = f'WP {self.path_index + 1}/{n_wp}' if n_wp > 0 else 'directo'
                self.get_logger().info(
                    f'[yolo_person] MOVING | {wp_str} ({xcl:.2f},{ycl:.2f}) | '
                    f'd={d:.2f} m | Fth={math.degrees(Fth):.1f} deg'
                )

            WAYPOINT_RADIUS = 0.4   # avanzar al siguiente waypoint cuando esté a <0.4 m

            if is_last_waypoint and d < ARRIVAL_DIST:
                self.current_speed = Twist()
                self.cmd_vel_pub.publish(self.current_speed)
                self.state = State.ARRIVED
                self._publish_state()
                self.get_logger().info(
                    f'[yolo_person] MOVING → ARRIVED | Goal alcanzado: ({xcl:.2f},{ycl:.2f})'
                )
            elif not is_last_waypoint and d < WAYPOINT_RADIUS:
                self.path_index += 1
                self.get_logger().info(
                    f'[yolo_person] MOVING | Waypoint alcanzado → '
                    f'{self.path_index + 1}/{len(self.path_waypoints)}'
                )
            elif self._trigger_wander:
                # Stuck too long – reverse + turn to escape
                self._trigger_wander    = False
                self._stuck_ticks       = 0
                self._noise_magnitude   = 0.0
                self._last_check_pos    = None
                self._pos_check_counter = 0
                self.current_speed      = Twist()
                self._wander_ticks_left = self._wander_duration
                self._wander_ang_sign   = random.choice([-1, 1])
                self.state = State.WANDERING
                self._publish_state()
                self.get_logger().info(
                    f'[yolo_person] MOVING → WANDERING (reversing) | '
                    f'Goal saved: ({xcl:.2f}, {ycl:.2f}) | '
                    f'Reversing {self._wander_duration // 25}s, turn sign={self._wander_ang_sign}'
                )
            else:
                self.current_speed = self._speed_behavior(self.current_speed, Fth)
                self.cmd_vel_pub.publish(self.current_speed)

        elif self.state == State.WANDERING:
            self._wander_ticks_left -= 1

            # Reverse + slight rotation to break out of local minimum
            twist = Twist()
            twist.linear.x  = -0.10                          # reverse
            twist.angular.z =  0.15 * self._wander_ang_sign  # rotate while reversing
            self.cmd_vel_pub.publish(twist)

            if self._wander_ticks_left <= 0:
                self.current_speed      = Twist()
                self._last_check_pos    = None
                self._stuck_ticks       = 0
                self._pos_check_counter = 0
                self.state = State.MOVING_AND_OBSTACLEDETECTION
                self._publish_state()
                self.get_logger().info(
                    f'[yolo_person] WANDERING → MOVING | '
                    f'Resuming goal: ({self.person_goal[0]:.2f}, {self.person_goal[1]:.2f})'
                )


# ---------------------------------------------------------------------------
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
