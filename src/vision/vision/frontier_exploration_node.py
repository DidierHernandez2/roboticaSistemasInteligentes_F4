#!/usr/bin/env python3
"""
frontier_exploration_node.py

El robot explora autónomamente buscando "fronteras": celdas libres
adyacentes a celdas desconocidas en el mapa de slam_toolbox.

Máquina de estados:
  SPINNING     – giro inicial 360° para que slam_toolbox tenga datos
  FINDING      – calcula fronteras y elige la mejor
  PATH_PLANNING– solicita camino A* al frontier goal
  MOVING       – sigue el camino, obstacle_avoidance filtra /cmd_vel
  ARRIVED      – llegó al frontier, busca el siguiente
  DONE         – no quedan fronteras, exploración completa

Publicaciones:
  /cmd_vel_desired        (Twist)           → obstacle_avoidance_node
  /frontier_markers       (MarkerArray)     → todos los clusters en RViz
  /frontier_goal_marker   (Marker)          → objetivo actual en RViz

Suscripciones:
  /map   (OccupancyGrid)
"""
import math
import random
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from enum import Enum
from collections import deque

from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import tf2_ros
from tf_transformations import euler_from_quaternion


# ---------------------------------------------------------------------------
# Attraction speed constants  (mismos que yolo_person_node)
# ---------------------------------------------------------------------------
LIN_ACC  = 0.0015
LIN_DES  = -0.0005
MAX_LIN_SPEED_HIGH = 0.20
MAX_LIN_SPEED_MID  = 0.08
ANG_ACC_LOW  = 0.001
ANG_ACC_MID  = 0.003
ANG_ACC_HIGH = 0.006
MAX_ANG_SPEED_LOW  = 0.15
MAX_ANG_SPEED_MID  = 0.25
MAX_ANG_SPEED_HIGH = 0.35
FORCE_THRESHOLD_LOW  = math.pi / 10
FORCE_THRESHOLD_MID  = math.pi / 2
FORCE_THRESHOLD_HIGH = math.pi * (4 / 5)
ARRIVAL_DIST    = 0.5    # m – distancia para considerar frontier alcanzado
WAYPOINT_RADIUS = 0.4   # m – avanzar al siguiente waypoint


# ---------------------------------------------------------------------------
class State(Enum):
    SPINNING      = 'SPINNING'
    FINDING       = 'FINDING'
    PATH_PLANNING = 'PATH_PLANNING'
    MOVING        = 'MOVING'
    ARRIVED       = 'ARRIVED'
    DONE          = 'DONE'


# ---------------------------------------------------------------------------
def _find_frontier_cells(grid: np.ndarray) -> list:
    """
    Devuelve lista de (row, col) de celdas frontera.
    Una celda es frontera si:
      - Es libre (0 <= valor < 50)
      - Tiene al menos un vecino 4-conectado desconocido (valor == -1)
    """
    rows, cols = grid.shape
    free     = (grid >= 0) & (grid < 50)
    unknown  = grid < 0   # -1

    # Desplazar unknown en 4 direcciones y OR
    up    = np.zeros_like(unknown)
    down  = np.zeros_like(unknown)
    left  = np.zeros_like(unknown)
    right = np.zeros_like(unknown)
    up[1:,  :]  = unknown[:-1, :]
    down[:-1, :] = unknown[1:,  :]
    left[:,  1:] = unknown[:,  :-1]
    right[:, :-1] = unknown[:, 1:]

    adj_unknown = up | down | left | right
    frontier_mask = free & adj_unknown

    frontier_cells = list(zip(*np.where(frontier_mask)))
    return [(int(r), int(c)) for r, c in frontier_cells]


def _cluster_frontiers(cells: list, min_size: int = 5) -> list:
    """
    Agrupa celdas frontera en clusters usando BFS 8-conectado.
    Descarta clusters menores de min_size celdas.
    Devuelve lista de listas de (row, col).
    """
    if not cells:
        return []

    cell_set = set(cells)
    visited  = set()
    clusters = []

    for seed in cells:
        if seed in visited:
            continue
        cluster = []
        q = deque([seed])
        visited.add(seed)
        while q:
            r, c = q.popleft()
            cluster.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (r + dr, c + dc)
                    if nb in cell_set and nb not in visited:
                        visited.add(nb)
                        q.append(nb)
        if len(cluster) >= min_size:
            clusters.append(cluster)

    return clusters


def _cluster_centroid(cluster: list) -> tuple:
    rows = [c[0] for c in cluster]
    cols = [c[1] for c in cluster]
    return (sum(rows) / len(rows), sum(cols) / len(cols))


def _farthest_cell(cluster: list, robot_row: float, robot_col: float) -> tuple:
    """
    Devuelve la celda del cluster más lejana al robot.
    Esto empuja al robot hacia el interior de la zona no explorada.
    """
    best_d = -1.0
    best   = cluster[0]
    for (r, c) in cluster:
        d = math.hypot(r - robot_row, c - robot_col)
        if d > best_d:
            best_d = d
            best   = (r, c)
    return best


# ---------------------------------------------------------------------------
class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')

        # Parámetros
        self.declare_parameter('spin_ang_vel',   0.15)   # rad/s giro inicial
        self.declare_parameter('min_cluster',    8)      # mín celdas por cluster
        self.declare_parameter('replan_interval', 5)     # s entre replans en MOVING

        self._spin_vel      = self.get_parameter('spin_ang_vel').get_parameter_value().double_value
        self._min_cluster   = self.get_parameter('min_cluster').get_parameter_value().integer_value
        replan_s            = self.get_parameter('replan_interval').get_parameter_value().integer_value

        # Estado
        self.state         = State.SPINNING
        self._current_speed = Twist()

        # Giro inicial: 360° / vel × 25 Hz
        self._spin_ticks_total = math.ceil(2 * math.pi / self._spin_vel / (1.0 / 25.0))
        self._spin_ticks_done  = 0
        self._spin_log         = 0

        # Mapa
        self._map: OccupancyGrid | None = None
        self._grid: np.ndarray  | None = None

        # Frontier actual
        self._goal: tuple | None = None   # (x, y) mundo
        self._clusters: list = []

        # Blacklist de fronteras ya visitadas (x, y) para no repetir
        self._visited_frontiers: list = []
        self._visited_radius = 0.5      # m — radio para considerar "ya visitado"

        # A* path
        self.path_waypoints: list = []
        self.path_index: int = 0
        self._path_future        = None
        self._path_request_sent  = False

        # Replan periódico
        self._REPLAN_INTERVAL = replan_s * 25
        self._replan_counter  = 0

        # Stuck detection
        self._last_check_pos    = None
        self._stuck_ticks       = 0
        self._stuck_threshold   = 3
        self._wander_threshold  = 7
        self._noise_magnitude   = 0.0
        self._pos_check_counter = 0
        self._trigger_wander    = False
        self._stuck_repulsors   = []
        self._wander_ticks_left = 0
        self._wander_duration   = 4 * 25
        self._wander_ang_sign   = 1

        # Logs
        self._move_log = 0

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        tl_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        path_qos = QoSProfile(depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)

        # Suscripciones
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, tl_qos)

        # Publicaciones
        self.cmd_vel_pub     = self.create_publisher(Twist,       '/cmd_vel_desired',      10)
        self.path_pub        = self.create_publisher(Path,        '/planned_path',          path_qos)
        self.frontier_pub    = self.create_publisher(MarkerArray, '/frontier_markers',      10)
        self.goal_marker_pub = self.create_publisher(Marker,      '/frontier_goal_marker',  10)

        # Servicio A*
        self.path_client = self.create_client(GetPlan, 'compute_path')

        # 25 Hz control loop
        self.create_timer(1.0 / 25.0, self.control_loop)
        # 1 Hz frontier visualization
        self.create_timer(1.0, self._publish_frontier_markers)

        self.get_logger().info('[frontier] Iniciado | Estado: SPINNING')

    # -----------------------------------------------------------------------
    # Map callback
    # -----------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg
        w, h = msg.info.width, msg.info.height
        raw  = np.array(msg.data, dtype=np.int16).reshape((h, w))
        self._grid = raw

    # -----------------------------------------------------------------------
    # TF helpers
    # -----------------------------------------------------------------------
    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            q = [t.transform.rotation.x, t.transform.rotation.y,
                 t.transform.rotation.z, t.transform.rotation.w]
            _, _, yaw = euler_from_quaternion(q)
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None

    # -----------------------------------------------------------------------
    # Frontier computation
    # -----------------------------------------------------------------------
    def _is_visited(self, wx: float, wy: float) -> bool:
        for (vx, vy) in self._visited_frontiers:
            if math.hypot(wx - vx, wy - vy) < self._visited_radius:
                return True
        return False

    def _compute_frontiers(self):
        """
        Calcula fronteras y devuelve clusters ordenados por score.
        El target de cada cluster es la celda MÁS LEJANA al robot,
        no el centroide — esto empuja al robot hacia zonas no exploradas.
        Los clusters cuyo target ya fue visitado se descartan.
        """
        if self._grid is None or self._map is None:
            return []

        cells    = _find_frontier_cells(self._grid)
        clusters = _cluster_frontiers(cells, self._min_cluster)

        if not clusters:
            return []

        pose = self._get_robot_pose()
        if pose is None:
            return []

        rx, ry, _ = pose
        info       = self._map.info

        # Posición del robot en celdas (para _farthest_cell)
        robot_col = (rx - info.origin.position.x) / info.resolution
        robot_row = (ry - info.origin.position.y) / info.resolution

        scored = []
        for cl in clusters:
            # Target = celda más lejana al robot en el cluster
            fr, fc = _farthest_cell(cl, robot_row, robot_col)
            wx = fc * info.resolution + info.origin.position.x + info.resolution / 2
            wy = fr * info.resolution + info.origin.position.y + info.resolution / 2

            # Descartar si ya fue visitado
            if self._is_visited(wx, wy):
                continue

            dist  = math.hypot(wx - rx, wy - ry)
            size  = len(cl)
            # Score: fronteras más grandes y más lejanas tienen prioridad
            score = size * math.log1p(dist)
            scored.append((score, cl, wx, wy))

        scored.sort(key=lambda x: x[0], reverse=True)
        return scored   # list of (score, cluster, wx, wy)

    def _world_to_cell(self, wx, wy):
        info = self._map.info
        col  = int((wx - info.origin.position.x) / info.resolution)
        row  = int((wy - info.origin.position.y) / info.resolution)
        return row, col

    # -----------------------------------------------------------------------
    # Attraction / speed (igual que yolo_person_node)
    # -----------------------------------------------------------------------
    def _check_stuck(self, x, y):
        self._pos_check_counter += 1
        if self._pos_check_counter < 25:
            return
        self._pos_check_counter = 0

        if self._last_check_pos is None:
            self._last_check_pos = (x, y)
            return

        dist = math.hypot(x - self._last_check_pos[0], y - self._last_check_pos[1])
        self._last_check_pos = (x, y)

        if dist < 0.05:
            self._stuck_ticks += 1
            if self._stuck_ticks >= self._wander_threshold:
                self._trigger_wander = True
                self._noise_magnitude = 0.0
                self._stuck_repulsors.append((x, y))
                self.get_logger().warn(
                    f'[frontier] STUCK {self._stuck_ticks}s → WANDERING')
            elif self._stuck_ticks >= self._stuck_threshold:
                self._noise_magnitude = min(
                    math.pi / 3,
                    (self._stuck_ticks - self._stuck_threshold + 1) * 0.15)
        else:
            self._stuck_ticks     = 0
            self._noise_magnitude = 0.0
            self._trigger_wander  = False

    def _calculate_attraction(self, xcl, ycl):
        pose = self._get_robot_pose()
        if pose is None:
            return None
        x, y, th = pose
        d = math.hypot(x - xcl, y - ycl)
        if d < 1e-6:
            d = 1e-6

        Fx = math.cos(math.atan2(ycl - y, xcl - x) - th)
        Fy = math.sin(math.atan2(ycl - y, xcl - x) - th)

        for (sx, sy) in self._stuck_repulsors:
            ds = math.hypot(x - sx, y - sy)
            if ds < 1e-3:
                ds = 1e-3
            if ds > 1.5:
                continue
            rep_angle = math.atan2(y - sy, x - sx) - th
            mag = 0.8 / (ds ** 2)
            Fx += mag * math.cos(rep_angle)
            Fy += mag * math.sin(rep_angle)

        Fth = math.atan2(Fy, Fx)
        self._check_stuck(x, y)
        if self._noise_magnitude > 0.0:
            Fth += random.gauss(0.0, self._noise_magnitude)
        return Fth, d

    def _speed_behavior(self, cur: Twist, Fth: float) -> Twist:
        out = Twist()
        if abs(Fth) < FORCE_THRESHOLD_LOW:
            out.linear.x  = min(cur.linear.x + LIN_ACC, MAX_LIN_SPEED_HIGH)
            out.angular.z = 0.0
        elif abs(Fth) < FORCE_THRESHOLD_MID:
            out.linear.x  = max(cur.linear.x + LIN_DES, MAX_LIN_SPEED_MID)
            out.angular.z = max(cur.angular.z + ANG_ACC_LOW * math.copysign(1, Fth),
                                MAX_ANG_SPEED_LOW * math.copysign(1, Fth))
        elif abs(Fth) < FORCE_THRESHOLD_HIGH:
            out.linear.x  = 0.0
            out.angular.z = max(cur.angular.z + ANG_ACC_MID * math.copysign(1, Fth),
                                MAX_ANG_SPEED_MID * math.copysign(1, Fth))
        else:
            out.linear.x  = 0.0
            out.angular.z = max(cur.angular.z + ANG_ACC_HIGH * math.copysign(1, Fth),
                                MAX_ANG_SPEED_HIGH * math.copysign(1, Fth))
        return out

    # -----------------------------------------------------------------------
    # Visualización de fronteras
    # -----------------------------------------------------------------------
    def _publish_frontier_markers(self):
        if self._grid is None or self._map is None:
            return

        scored   = self._compute_frontiers()
        ma       = MarkerArray()
        info     = self._map.info

        # Borrar markers anteriores
        del_m        = Marker()
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        for idx, (score, cluster, wx, wy) in enumerate(scored):
            m             = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns              = 'frontiers'
            m.id              = idx
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            sz = max(0.15, min(0.5, len(cluster) * 0.01))
            m.scale.x = m.scale.y = m.scale.z = sz

            # Objetivo actual → verde, resto → amarillo
            if self._goal is not None and abs(wx - self._goal[0]) < 0.1:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
            else:
                m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)

            ma.markers.append(m)

        self.frontier_pub.publish(ma)

    def _publish_goal_marker(self):
        if self._goal is None:
            return
        m                    = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'frontier_goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = self._goal[0]
        m.pose.position.y    = self._goal[1]
        m.pose.position.z    = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color              = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        m.lifetime.sec       = 0
        self.goal_marker_pub.publish(m)

    # -----------------------------------------------------------------------
    # Control loop 25 Hz
    # -----------------------------------------------------------------------
    def control_loop(self):

        # ── SPINNING ────────────────────────────────────────────────
        if self.state == State.SPINNING:
            twist = Twist()
            twist.angular.z = self._spin_vel
            self.cmd_vel_pub.publish(twist)

            self._spin_ticks_done += 1
            self._spin_log        += 1
            if self._spin_log >= 25:
                self._spin_log = 0
                pct = self._spin_ticks_done / self._spin_ticks_total * 100
                self.get_logger().info(
                    f'[frontier] SPINNING {pct:.0f}% | '
                    f'mapa: {"OK" if self._grid is not None else "esperando..."}'
                )

            if self._spin_ticks_done >= self._spin_ticks_total:
                self.cmd_vel_pub.publish(Twist())
                self.state = State.FINDING
                self.get_logger().info('[frontier] SPINNING completo → FINDING')

        # ── FINDING ─────────────────────────────────────────────────
        elif self.state == State.FINDING:
            self.cmd_vel_pub.publish(Twist())

            scored = self._compute_frontiers()
            if not scored:
                self.get_logger().info('[frontier] No hay fronteras → DONE')
                self.state = State.DONE
                return

            _, cluster, wx, wy = scored[0]
            self._goal = (wx, wy)
            self._path_request_sent = False
            self._path_future       = None
            self.path_waypoints     = []
            self.path_index         = 0
            self._replan_counter    = 0
            self._last_check_pos    = None
            self._stuck_ticks       = 0
            self._noise_magnitude   = 0.0
            self._pos_check_counter = 0

            self._publish_goal_marker()
            self.state = State.PATH_PLANNING
            self.get_logger().info(
                f'[frontier] FINDING → PATH_PLANNING | '
                f'Frontier en ({wx:.2f}, {wy:.2f}) | '
                f'cluster size={len(cluster)}'
            )

        # ── PATH_PLANNING ────────────────────────────────────────────
        elif self.state == State.PATH_PLANNING:
            self.cmd_vel_pub.publish(Twist())

            if not self._path_request_sent:
                pose = self._get_robot_pose()
                if pose is None:
                    return
                x, y, _ = pose

                if not self.path_client.service_is_ready():
                    return

                req = GetPlan.Request()
                req.start.header.frame_id = 'map'
                req.start.pose.position.x = x
                req.start.pose.position.y = y
                req.goal.header.frame_id  = 'map'
                req.goal.pose.position.x  = self._goal[0]
                req.goal.pose.position.y  = self._goal[1]

                self._path_future       = self.path_client.call_async(req)
                self._path_request_sent = True
                self.get_logger().info(
                    f'[frontier] PATH_PLANNING | A* '
                    f'({x:.2f},{y:.2f}) → ({self._goal[0]:.2f},{self._goal[1]:.2f})'
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
                        f'[frontier] PATH_PLANNING → MOVING | '
                        f'{len(self.path_waypoints)} waypoints'
                    )
                else:
                    # Sin camino: navegar directo o saltar a siguiente frontier
                    self.path_waypoints = [self._goal]
                    self.get_logger().warn(
                        '[frontier] A* sin camino — navegación directa'
                    )

                self.path_index         = 0
                self._path_request_sent = False
                self._path_future       = None
                self._current_speed     = Twist()
                self.state = State.MOVING

        # ── MOVING ───────────────────────────────────────────────────
        elif self.state == State.MOVING:
            if self._goal is None:
                return

            # Replan periódico
            self._replan_counter += 1
            if self._replan_counter >= self._REPLAN_INTERVAL:
                self._replan_counter    = 0
                self._path_request_sent = False
                self._path_future       = None
                self.state = State.PATH_PLANNING
                self.get_logger().info(
                    '[frontier] MOVING → PATH_PLANNING | Replan con mapa actualizado'
                )
                return

            is_last = (not self.path_waypoints or
                       self.path_index >= len(self.path_waypoints) - 1)
            if self.path_waypoints and self.path_index < len(self.path_waypoints):
                xcl, ycl = self.path_waypoints[self.path_index]
            else:
                xcl, ycl = self._goal

            result = self._calculate_attraction(xcl, ycl)
            if result is None:
                return
            Fth, d = result

            self._move_log += 1
            if self._move_log >= 25:
                self._move_log = 0
                n_wp   = len(self.path_waypoints)
                wp_str = f'WP {self.path_index + 1}/{n_wp}' if n_wp else 'directo'
                self.get_logger().info(
                    f'[frontier] MOVING | {wp_str} ({xcl:.2f},{ycl:.2f}) | '
                    f'd={d:.2f} m | Fth={math.degrees(Fth):.1f}°'
                )

            if is_last and d < ARRIVAL_DIST:
                self._current_speed = Twist()
                self.cmd_vel_pub.publish(self._current_speed)
                self.state = State.ARRIVED
                self.get_logger().info(
                    f'[frontier] MOVING → ARRIVED | ({xcl:.2f},{ycl:.2f})'
                )
            elif not is_last and d < WAYPOINT_RADIUS:
                self.path_index += 1
            elif self._trigger_wander:
                self._trigger_wander    = False
                self._stuck_ticks       = 0
                self._noise_magnitude   = 0.0
                self._last_check_pos    = None
                self._pos_check_counter = 0
                self._current_speed     = Twist()
                self._wander_ticks_left = self._wander_duration
                self._wander_ang_sign   = random.choice([-1, 1])
                # Después del wander vuelve a buscar frontier
                self.state = State.FINDING
                self.get_logger().info('[frontier] MOVING → FINDING (post-wander)')
                # Ejecutar wander inline (simplificado: pausa y gira)
            else:
                self._current_speed = self._speed_behavior(self._current_speed, Fth)
                self.cmd_vel_pub.publish(self._current_speed)

        # ── ARRIVED ──────────────────────────────────────────────────
        elif self.state == State.ARRIVED:
            self.cmd_vel_pub.publish(Twist())
            # Añadir frontier actual a la blacklist para no repetirlo
            if self._goal is not None:
                self._visited_frontiers.append(self._goal)
                self.get_logger().info(
                    f'[frontier] Frontier ({self._goal[0]:.2f},{self._goal[1]:.2f}) '
                    f'añadido a blacklist [{len(self._visited_frontiers)} visitados]'
                )
            self.state = State.FINDING
            self.get_logger().info('[frontier] ARRIVED → FINDING | Buscando siguiente frontier')

        # ── DONE ─────────────────────────────────────────────────────
        elif self.state == State.DONE:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('[frontier] Exploración completa.', throttle_duration_sec=10.0)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
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
