#!/usr/bin/env python3
"""
path_planning_node.py

Servicio ROS2 que implementa A* sobre el mapa de ocupación de slam_toolbox.

Servicio:  /compute_path  (nav_msgs/srv/GetPlan)
  request:  start (PoseStamped), goal (PoseStamped)
  response: plan  (nav_msgs/Path)  — waypoints en frame 'map'

Publica:   /planned_path  (nav_msgs/Path)  — para visualización en RViz
"""
import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# A* sobre grid 2D
# ---------------------------------------------------------------------------
def _inflate(grid: np.ndarray, radius: int) -> np.ndarray:
    """Dilata las celdas ocupadas (>=50) por 'radius' celdas en todas direcciones."""
    result = grid.copy()
    rows, cols = grid.shape
    occ_r, occ_c = np.where(grid >= 50)
    for r, c in zip(occ_r.tolist(), occ_c.tolist()):
        r0 = max(0, r - radius)
        r1 = min(rows, r + radius + 1)
        c0 = max(0, c - radius)
        c1 = min(cols, c + radius + 1)
        result[r0:r1, c0:c1] = 100
    return result


def _astar(grid: np.ndarray, start: tuple, goal: tuple):
    """
    A* 8-direccional.
    grid: numpy int8/int16 — celda libre si < 50, ocupada si >= 50
    start/goal: (row, col)
    Devuelve lista de (row, col) o None si no hay camino.
    """
    rows, cols = grid.shape

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # open_set: (f, g, cell)
    open_set = [(h(start, goal), 0.0, start)]
    came_from: dict = {}
    g_score: dict = {start: 0.0}

    while open_set:
        f, g, cur = heapq.heappop(open_set)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return path

        if g > g_score.get(cur, float('inf')) + 1e-9:
            continue  # entrada obsoleta

        r, c = cur
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if grid[nr, nc] >= 50:
                    continue
                ng = g + math.hypot(dr, dc)
                nb = (nr, nc)
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_set, (ng + h(nb, goal), ng, nb))

    return None


def _nearest_free(grid: np.ndarray, row: int, col: int, max_r: int = 10):
    """Devuelve la celda libre más cercana a (row, col), buscando en radio max_r."""
    rows, cols = grid.shape
    for radius in range(max_r + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if abs(dr) != radius and abs(dc) != radius:
                    continue
                nr, nc = row + dr, col + dc
                if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] < 50:
                    return (nr, nc)
    return None


# ---------------------------------------------------------------------------
class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.declare_parameter('inflation_radius', 4)   # celdas de inflado de obstáculos
        self.declare_parameter('waypoint_step',    8)   # submuestreo: 1 waypoint cada N celdas
        self.declare_parameter('map_topic',        '/map')

        self._inflation_radius = self.get_parameter('inflation_radius').get_parameter_value().integer_value
        self._waypoint_step    = self.get_parameter('waypoint_step').get_parameter_value().integer_value
        map_topic              = self.get_parameter('map_topic').get_parameter_value().string_value

        self._map: OccupancyGrid | None = None
        self._grid_inflated: np.ndarray | None = None

        # QoS transient-local para recibir el mapa aunque nos suscribamos tarde
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)

        self._srv = self.create_service(GetPlan, 'compute_path', self._plan_cb)

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._path_pub = self.create_publisher(Path, '/planned_path', path_qos)

        self.get_logger().info(
            f'[path_planning] Listo | inflation={self._inflation_radius} celdas | '
            f'esperando {map_topic}...'
        )

    # -----------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg
        w, h = msg.info.width, msg.info.height
        raw = np.array(msg.data, dtype=np.int16).reshape((h, w))
        # desconocido (-1) → ocupado para planificación
        passable = raw.copy()
        passable[raw < 0] = 100
        self._grid_inflated = _inflate(passable, self._inflation_radius)
        self.get_logger().info(
            f'[path_planning] Mapa actualizado: {w}x{h} celdas, '
            f'res={msg.info.resolution:.3f} m/celda'
        )

    def _world_to_cell(self, wx: float, wy: float) -> tuple:
        info = self._map.info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        return row, col

    def _cell_to_world(self, row: int, col: int) -> tuple:
        info = self._map.info
        wx = col * info.resolution + info.origin.position.x + info.resolution / 2.0
        wy = row * info.resolution + info.origin.position.y + info.resolution / 2.0
        return wx, wy

    # -----------------------------------------------------------------------
    def _plan_cb(self, request: GetPlan.Request, response: GetPlan.Response):
        if self._map is None or self._grid_inflated is None:
            self.get_logger().warn('[path_planning] Aún no hay mapa disponible')
            return response

        sx = request.start.pose.position.x
        sy = request.start.pose.position.y
        gx = request.goal.pose.position.x
        gy = request.goal.pose.position.y

        start_cell = self._world_to_cell(sx, sy)
        goal_cell  = self._world_to_cell(gx, gy)

        rows, cols = self._grid_inflated.shape

        # Validar y ajustar start
        if not (0 <= start_cell[0] < rows and 0 <= start_cell[1] < cols):
            self.get_logger().warn(f'[path_planning] Start {start_cell} fuera del mapa')
            return response
        if self._grid_inflated[start_cell] >= 50:
            free = _nearest_free(self._grid_inflated, start_cell[0], start_cell[1])
            if free is None:
                self.get_logger().warn('[path_planning] No hay celda libre cerca del start')
                return response
            self.get_logger().info(f'[path_planning] Start ajustado {start_cell} → {free}')
            start_cell = free

        # Validar y ajustar goal
        if not (0 <= goal_cell[0] < rows and 0 <= goal_cell[1] < cols):
            self.get_logger().warn(f'[path_planning] Goal {goal_cell} fuera del mapa')
            return response
        if self._grid_inflated[goal_cell] >= 50:
            free = _nearest_free(self._grid_inflated, goal_cell[0], goal_cell[1])
            if free is None:
                self.get_logger().warn('[path_planning] No hay celda libre cerca del goal')
                return response
            self.get_logger().info(f'[path_planning] Goal ajustado {goal_cell} → {free}')
            goal_cell = free

        self.get_logger().info(
            f'[path_planning] A* | start={start_cell} goal={goal_cell}'
        )

        cell_path = _astar(self._grid_inflated, start_cell, goal_cell)

        if cell_path is None:
            self.get_logger().warn('[path_planning] A* no encontró camino')
            return response

        # Construir Path submuestreado
        path_msg = Path()
        path_msg.header.frame_id = self._map.header.frame_id or 'map'
        path_msg.header.stamp    = self.get_clock().now().to_msg()

        step = max(1, self._waypoint_step)
        for cell in cell_path[::step]:
            wx, wy = self._cell_to_world(cell[0], cell[1])
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            path_msg.poses.append(ps)

        # Siempre incluir el goal exacto como último waypoint
        final = PoseStamped()
        final.header = path_msg.header
        final.pose.position.x = gx
        final.pose.position.y = gy
        path_msg.poses.append(final)

        response.plan = path_msg
        self._path_pub.publish(path_msg)

        self.get_logger().info(
            f'[path_planning] Camino encontrado: {len(cell_path)} celdas → '
            f'{len(path_msg.poses)} waypoints'
        )
        return response


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
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
