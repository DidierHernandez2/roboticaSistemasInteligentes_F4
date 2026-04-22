#!/usr/bin/env python3
"""
laser_map_node.py

Acumula todos los puntos de impacto del LIDAR en frame 'map' y los
publica como PointCloud2 persistente — los puntos nunca desaparecen.

Suscripciones:
  /scan  (sensor_msgs/LaserScan)

Publicaciones:
  /laser_map  (sensor_msgs/PointCloud2)  — nube de puntos acumulada en frame map
"""
import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros


class LaserMapNode(Node):
    def __init__(self):
        super().__init__('laser_map_node')

        self.declare_parameter('quantize_m', 0.03)   # resolución de deduplicación (3 cm)
        self.declare_parameter('publish_hz', 2.0)

        self._q       = self.get_parameter('quantize_m').get_parameter_value().double_value
        pub_hz        = self.get_parameter('publish_hz').get_parameter_value().double_value

        # Set de celdas ocupadas: (int_x, int_y) cuantizados
        self._hits: set = set()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._pub = self.create_publisher(PointCloud2, '/laser_map', 10)

        self.create_timer(1.0 / pub_hz, self._publish)

        self.get_logger().info(
            f'[laser_map] Listo | quantize={self._q} m | publish={pub_hz} Hz'
        )

    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        q  = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        angle = msg.angle_min
        for r in msg.ranges:
            angle += msg.angle_increment
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r >= msg.range_max:
                continue

            beam = yaw + angle
            wx = tx + r * math.cos(beam)
            wy = ty + r * math.sin(beam)

            # Cuantizar para evitar duplicados y limitar memoria
            ix = int(wx / self._q)
            iy = int(wy / self._q)
            self._hits.add((ix, iy))

    # ------------------------------------------------------------------
    def _publish(self):
        if not self._hits:
            return

        # Construir PointCloud2 mínimo: campos X, Y, Z (Z=0)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        hits = list(self._hits)
        data = bytearray()
        for (ix, iy) in hits:
            data += struct.pack('fff', ix * self._q, iy * self._q, 0.0)

        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.height          = 1
        msg.width           = len(hits)
        msg.fields          = fields
        msg.is_bigendian    = False
        msg.point_step      = 12   # 3 × float32
        msg.row_step        = 12 * len(hits)
        msg.data            = bytes(data)
        msg.is_dense        = True

        self._pub.publish(msg)
        self.get_logger().debug(f'[laser_map] {len(hits)} puntos acumulados')


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LaserMapNode()
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
