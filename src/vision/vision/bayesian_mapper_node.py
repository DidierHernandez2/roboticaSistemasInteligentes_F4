#!/usr/bin/env python3
"""
bayesian_mapper_node.py

Construye un mapa de ocupación bayesiano desde cero usando:
  - /scan  (LaserScan)   — datos del LIDAR
  - TF map → base_link   — pose del robot en cada instante
  - /robot_state         — solo actualiza en MAPPING y MOVING

Algoritmo: log-odds update con ray casting (Bresenham) por cada rayo.

Publicaciones:
  /bayesian_map  (nav_msgs/OccupancyGrid)  — mapa propio, visible como grid

Parámetros:
  resolution   (m/celda)   — tamaño de cada cuadro (default 0.10 m)
  map_width_m  (m)         — ancho total del mapa en metros (default 30)
  map_height_m (m)         — alto total del mapa en metros  (default 30)
  publish_hz               — frecuencia de publicación (default 2 Hz)
  l_occ                    — incremento log-odds al detectar obstáculo
  l_free                   — decremento log-odds al detectar celda libre
  l_min / l_max            — saturación del log-odds
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import tf2_ros


# Estados en los que el nodo actualiza el mapa
ACTIVE_STATES = {'MAPPING', 'MOVING_AND_OBSTACLEDETECTION', 'WANDERING'}


def _bresenham(r0: int, c0: int, r1: int, c1: int):
    """
    Genera las celdas intermedias entre (r0,c0) y (r1,c1) usando Bresenham.
    Devuelve lista de (row, col) SIN incluir el punto final.
    """
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while (r, c) != (r1, c1):
        cells.append((r, c))
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r   += sr
        if e2 <  dr:
            err += dr
            c   += sc
    return cells


class BayesianMapperNode(Node):
    def __init__(self):
        super().__init__('bayesian_mapper_node')

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('resolution',    0.10)
        self.declare_parameter('map_width_m',  30.0)
        self.declare_parameter('map_height_m', 30.0)
        self.declare_parameter('publish_hz',    2.0)
        self.declare_parameter('l_occ',         0.85)
        self.declare_parameter('l_free',       -0.40)
        self.declare_parameter('l_min',        -5.0)
        self.declare_parameter('l_max',         5.0)

        res      = self.get_parameter('resolution').get_parameter_value().double_value
        w_m      = self.get_parameter('map_width_m').get_parameter_value().double_value
        h_m      = self.get_parameter('map_height_m').get_parameter_value().double_value
        pub_hz   = self.get_parameter('publish_hz').get_parameter_value().double_value
        self._l_occ  = self.get_parameter('l_occ').get_parameter_value().double_value
        self._l_free = self.get_parameter('l_free').get_parameter_value().double_value
        self._l_min  = self.get_parameter('l_min').get_parameter_value().double_value
        self._l_max  = self.get_parameter('l_max').get_parameter_value().double_value

        self._res  = res
        self._cols = int(w_m / res)
        self._rows = int(h_m / res)
        # Origen del mapa: centrado en (0,0) del mundo
        self._ox = -w_m / 2.0
        self._oy = -h_m / 2.0

        # Log-odds grid, inicializado a 0 (prior 0.5 = desconocido)
        self._logodds = np.zeros((self._rows, self._cols), dtype=np.float32)

        # Estado actual del robot
        self._state = ''

        # ── TF ────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS transient-local para el mapa ─────────────────────────
        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Suscripciones ─────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',        self._scan_cb,  10)
        self.create_subscription(String,    '/robot_state', self._state_cb, 10)

        # ── Publicador ────────────────────────────────────────────────
        self._map_pub = self.create_publisher(OccupancyGrid, '/bayesian_map', tl_qos)

        # ── Timer de publicación ──────────────────────────────────────
        self.create_timer(1.0 / pub_hz, self._publish_map)

        self.get_logger().info(
            f'[bayesian_mapper] Listo | '
            f'grid {self._cols}x{self._rows} celdas | '
            f'res={res} m/celda | '
            f'mapa {w_m}x{h_m} m'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: String):
        self._state = msg.data

    def _scan_cb(self, msg: LaserScan):
        if self._state not in ACTIVE_STATES:
            return

        # Obtener pose del robot en frame map
        try:
            t = self._tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        q  = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Celda del robot
        robot_col = int((rx - self._ox) / self._res)
        robot_row = int((ry - self._oy) / self._res)

        if not (0 <= robot_row < self._rows and 0 <= robot_col < self._cols):
            return   # robot fuera del mapa

        # ── Ray casting por cada rayo del LIDAR ───────────────────────
        angle = msg.angle_min
        for r in msg.ranges:
            angle += msg.angle_increment

            # Ignorar rangos inválidos
            if math.isnan(r) or math.isinf(r) or r < msg.range_min:
                continue

            hit = r < msg.range_max   # True = impacto real, False = rayo libre

            # Punto de impacto (o extremo del rayo si no hay hit)
            beam_angle = yaw + angle
            end_x = rx + r * math.cos(beam_angle)
            end_y = ry + r * math.sin(beam_angle)

            end_col = int((end_x - self._ox) / self._res)
            end_row = int((end_y - self._oy) / self._res)

            # Celdas libres a lo largo del rayo (Bresenham, sin el extremo)
            free_cells = _bresenham(robot_row, robot_col, end_row, end_col)
            for (fr, fc) in free_cells:
                if 0 <= fr < self._rows and 0 <= fc < self._cols:
                    self._logodds[fr, fc] = np.clip(
                        self._logodds[fr, fc] + self._l_free,
                        self._l_min, self._l_max
                    )

            # Celda de impacto → ocupada
            if hit and 0 <= end_row < self._rows and 0 <= end_col < self._cols:
                self._logodds[end_row, end_col] = np.clip(
                    self._logodds[end_row, end_col] + self._l_occ,
                    self._l_min, self._l_max
                )

    # ──────────────────────────────────────────────────────────────────
    # Publicación
    # ──────────────────────────────────────────────────────────────────
    def _publish_map(self):
        # Convertir log-odds → probabilidad → valor OccupancyGrid [0,100]
        # log-odds = 0  →  P = 0.5  →  valor = -1 (desconocido)
        # log-odds > 0  →  P > 0.5  →  obstáculo (100)
        # log-odds < 0  →  P < 0.5  →  libre (0)
        prob = 1.0 - 1.0 / (1.0 + np.exp(self._logodds))

        grid = np.full((self._rows, self._cols), -1, dtype=np.int8)
        grid[self._logodds > 0]  = np.clip(
            (prob[self._logodds > 0] * 100).astype(np.int8), 1, 100)
        grid[self._logodds < 0]  = 0
        # Celdas con log-odds == 0 se quedan como -1 (desconocidas)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.info.resolution = self._res
        msg.info.width      = self._cols
        msg.info.height     = self._rows
        msg.info.origin     = Pose(
            position=Point(x=self._ox, y=self._oy, z=0.0),
            orientation=Quaternion(w=1.0)
        )

        msg.data = grid.flatten().tolist()
        self._map_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = BayesianMapperNode()
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
