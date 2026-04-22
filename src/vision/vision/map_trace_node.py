#!/usr/bin/env python3
"""
map_trace_node.py

Publica un OccupancyGrid (/robot_trace) que muestra las celdas
visitadas por el robot durante la misión, coloreadas por estado:

  30  → MAPPING     (cyan claro en costmap)
  60  → SEARCHING   (azul en costmap)
  90  → MOVING      (naranja/rojo en costmap)

Las celdas no visitadas se publican como -1 (transparentes en RViz).
Una celda solo puede subir de valor, nunca bajar:
  MAPPING(30) < SEARCHING(60) < MOVING(90)

Suscripciones:
  /map          (nav_msgs/OccupancyGrid)    — dimensiones y metadata del mapa
  /robot_state  (std_msgs/String)           — estado de yolo_person_node
  /person_goal  (geometry_msgs/PointStamped)— goal publicado por yolo_person_node

Publicaciones:
  /robot_trace         (nav_msgs/OccupancyGrid)   — rastro coloreado por estado
  /person_goal_marker  (visualization_msgs/Marker) — esfera magenta en el goal
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf2_ros

# Valor de celda por estado — solo los estados que generan traza
STATE_VALUE = {
    'MAPPING':                      30,
    'SEARCHING':                    60,
    'MOVING_AND_OBSTACLEDETECTION': 90,
    'WANDERING':                    90,
}


class MapTraceNode(Node):
    def __init__(self):
        super().__init__('map_trace_node')

        self._map_info = None
        self._trace    = None   # numpy int8 array (height x width), init -1
        self._state    = 'SEARCHING'
        self._goal     = None   # (x, y) en frame map

        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, '/map',         self._map_cb,   tl_qos)
        self.create_subscription(String,        '/robot_state', self._state_cb, 10)
        self.create_subscription(PointStamped,  '/person_goal', self._goal_cb,  10)

        self._trace_pub  = self.create_publisher(OccupancyGrid, '/robot_trace',        tl_qos)
        self._marker_pub = self.create_publisher(Marker,        '/person_goal_marker', 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 5 Hz: muestrear posición y pintar celda
        self.create_timer(0.2, self._update_trace)
        # 2 Hz: publicar el grid acumulado
        self.create_timer(0.5, self._publish_trace)

        self.get_logger().info('[map_trace] Listo | esperando /map y /robot_state...')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        info = msg.info
        # Reiniciar traza solo si las dimensiones del mapa cambian
        if (self._map_info is None
                or info.width  != self._map_info.width
                or info.height != self._map_info.height):
            self._map_info = info
            self._trace    = np.full((info.height, info.width), -1, dtype=np.int8)
            self.get_logger().info(
                f'[map_trace] Mapa recibido: {info.width}x{info.height} celdas | '
                f'res={info.resolution:.3f} m/celda'
            )

    def _state_cb(self, msg: String):
        self._state = msg.data

    def _goal_cb(self, msg: PointStamped):
        self._goal = (msg.point.x, msg.point.y)
        self._publish_goal_marker()

    # ------------------------------------------------------------------
    # Marker del goal
    # ------------------------------------------------------------------
    def _publish_goal_marker(self):
        if self._goal is None:
            return
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'person_goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = self._goal[0]
        m.pose.position.y    = self._goal[1]
        m.pose.position.z    = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        # Magenta brillante para distinguirlo del camino verde
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime.sec = 0   # permanente hasta nuevo goal
        self._marker_pub.publish(m)

    # ------------------------------------------------------------------
    # Traza
    # ------------------------------------------------------------------
    def _update_trace(self):
        if self._map_info is None or self._trace is None:
            return

        value = STATE_VALUE.get(self._state)
        if value is None:
            return   # estados sin traza: PATH_PLANNING, PERSON_FOUND, ARRIVED, etc.

        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return

        wx = t.transform.translation.x
        wy = t.transform.translation.y

        info = self._map_info
        col  = int((wx - info.origin.position.x) / info.resolution)
        row  = int((wy - info.origin.position.y) / info.resolution)

        if 0 <= row < info.height and 0 <= col < info.width:
            current = int(self._trace[row, col])
            # Solo subir de valor (MAPPING < SEARCHING < MOVING)
            if current == -1 or value > current:
                self._trace[row, col] = value

    def _publish_trace(self):
        if self._map_info is None or self._trace is None:
            return

        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.header.stamp    = self.get_clock().now().to_msg()
        out.info            = self._map_info
        out.data            = self._trace.flatten().tolist()
        self._trace_pub.publish(out)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MapTraceNode()
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
