#!/usr/bin/env python3
"""
potential_field_viz_node.py

Publica un OccupancyGrid en /potential_field_map que muestra el campo
potencial combinado (atracción al goal + repulsión de obstáculos) sobre
el mapa de slam_toolbox.

Suscripciones:
  /map           (nav_msgs/OccupancyGrid)  — mapa de slam_toolbox
  /person_goal   (geometry_msgs/PointStamped) — goal publicado por yolo_person_node

Publicaciones:
  /potential_field_map  (nav_msgs/OccupancyGrid)  — campo potencial (0=bajo, 100=alto)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

try:
    from scipy.ndimage import distance_transform_edt
    _SCIPY = True
except ImportError:
    _SCIPY = False


def _distance_transform(binary_obstacle: np.ndarray) -> np.ndarray:
    """
    Devuelve la distancia (en celdas) de cada celda al obstáculo más cercano.
    binary_obstacle: True donde hay obstáculo.
    """
    if _SCIPY:
        return distance_transform_edt(~binary_obstacle).astype(np.float32)
    # Fallback: BFS desde todas las celdas obstáculo simultáneamente
    from collections import deque
    dist = np.full(binary_obstacle.shape, np.inf, dtype=np.float32)
    q = deque()
    rows, cols = binary_obstacle.shape
    for r, c in zip(*np.where(binary_obstacle)):
        dist[r, c] = 0.0
        q.append((r, c))
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < rows and 0 <= nc < cols:
                nd = dist[r, c] + 1.0
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    q.append((nr, nc))
    return dist


class PotentialFieldVizNode(Node):
    def __init__(self):
        super().__init__('potential_field_viz_node')

        # Parámetros
        self.declare_parameter('k_att',       1.0)    # ganancia atracción
        self.declare_parameter('k_rep',       3.0)    # ganancia repulsión
        self.declare_parameter('influence_m', 1.5)    # radio de influencia obstáculos (m)
        self.declare_parameter('update_hz',   1.0)    # frecuencia de actualización

        self._k_att      = self.get_parameter('k_att').get_parameter_value().double_value
        self._k_rep      = self.get_parameter('k_rep').get_parameter_value().double_value
        self._influence  = self.get_parameter('influence_m').get_parameter_value().double_value
        update_hz        = self.get_parameter('update_hz').get_parameter_value().double_value

        self._map:  OccupancyGrid | None = None
        self._goal: tuple | None         = None   # (x, y) en frame map

        # QoS transient-local para mapa
        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, tl_qos)
        self.create_subscription(PointStamped,  '/person_goal', self._goal_cb, 10)

        self._pub = self.create_publisher(OccupancyGrid, '/potential_field_map', tl_qos)

        self.create_timer(1.0 / update_hz, self._update)

        mode = 'scipy' if _SCIPY else 'BFS'
        self.get_logger().info(
            f'[pot_field_viz] Listo | dist_transform={mode} | '
            f'k_att={self._k_att} k_rep={self._k_rep} influence={self._influence}m'
        )

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _goal_cb(self, msg: PointStamped):
        self._goal = (msg.point.x, msg.point.y)
        self.get_logger().info(
            f'[pot_field_viz] Goal recibido: ({self._goal[0]:.2f}, {self._goal[1]:.2f})'
        )

    # ------------------------------------------------------------------
    def _update(self):
        if self._map is None:
            return

        info = self._map.info
        w, h = info.width, info.height
        res  = info.resolution

        raw = np.array(self._map.data, dtype=np.int16).reshape((h, w))

        # Obstáculos: celdas con valor >= 50 o desconocidas (-1)
        obstacle = (raw >= 50) | (raw < 0)

        # ── Repulsión: distancia al obstáculo más cercano ──────────────
        dist_obs = _distance_transform(obstacle) * res   # metros

        q_star = self._influence
        rep = np.zeros((h, w), dtype=np.float32)
        mask = (dist_obs > 0) & (dist_obs <= q_star)
        rep[mask] = 0.5 * self._k_rep * (1.0 / dist_obs[mask] - 1.0 / q_star) ** 2
        rep[obstacle] = rep[mask].max() if mask.any() else 0.0

        # ── Atracción: distancia al goal ───────────────────────────────
        att = np.zeros((h, w), dtype=np.float32)
        if self._goal is not None:
            gx, gy = self._goal
            ox = info.origin.position.x
            oy = info.origin.position.y
            col_grid = (np.arange(w) * res + ox + res / 2.0).reshape(1, w)
            row_grid = (np.arange(h) * res + oy + res / 2.0).reshape(h, 1)
            d_goal = np.sqrt((col_grid - gx) ** 2 + (row_grid - gy) ** 2)
            att = 0.5 * self._k_att * d_goal.astype(np.float32)

        # ── Campo combinado ────────────────────────────────────────────
        combined = att + rep

        # Normalizar a [0, 100], celdas desconocidas/obstáculo → -1
        v_min = combined[~obstacle].min() if (~obstacle).any() else 0.0
        v_max = combined[~obstacle].max() if (~obstacle).any() else 1.0
        if v_max - v_min < 1e-6:
            v_max = v_min + 1.0

        normalized = np.zeros((h, w), dtype=np.int8)
        free = ~obstacle
        normalized[free] = np.clip(
            ((combined[free] - v_min) / (v_max - v_min) * 98).astype(np.int8),
            0, 98
        )
        normalized[obstacle] = -1   # desconocido/obstáculo → gris en RViz

        # ── Publicar ───────────────────────────────────────────────────
        out = OccupancyGrid()
        out.header.frame_id = self._map.header.frame_id or 'map'
        out.header.stamp    = self.get_clock().now().to_msg()
        out.info            = info
        out.data            = normalized.flatten().tolist()
        self._pub.publish(out)


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldVizNode()
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
