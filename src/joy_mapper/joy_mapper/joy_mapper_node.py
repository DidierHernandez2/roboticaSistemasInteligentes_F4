#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoyMapperNode(Node):
    def __init__(self):
        super().__init__('joy_mapper_node')

        # Parámetros para mapear botones/ejes
        self.declare_parameter('axis_speed', 1)      # stick vertical
        self.declare_parameter('btn_run', 0)         # A
        self.declare_parameter('btn_stop', 1)        # B
        self.declare_parameter('btn_reverse', 2)     # X
        self.declare_parameter('btn_status', 3)      # Y
        self.declare_parameter('deadzone', 0.2)
        self.declare_parameter('min_hz', 0.0)
        self.declare_parameter('max_hz', 60.0)
        self.declare_parameter('step_hz', 1.0)

        self.axis_speed = self.get_parameter('axis_speed').value
        self.btn_run = self.get_parameter('btn_run').value
        self.btn_stop = self.get_parameter('btn_stop').value
        self.btn_reverse = self.get_parameter('btn_reverse').value
        self.btn_status = self.get_parameter('btn_status').value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.min_hz = float(self.get_parameter('min_hz').value)
        self.max_hz = float(self.get_parameter('max_hz').value)
        self.step_hz = float(self.get_parameter('step_hz').value)

        self.last_buttons = []
        self.target_hz = 10.0

        self.pub_cmd = self.create_publisher(String, '/l510_cmd', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('joy_mapper_node listo')

    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.pub_cmd.publish(msg)
        self.get_logger().info(f'Publicado /l510_cmd: {text}')

    def rising_edge(self, buttons, idx):
        if idx >= len(buttons):
            return False
        old = self.last_buttons[idx] if idx < len(self.last_buttons) else 0
        return buttons[idx] == 1 and old == 0

    def joy_callback(self, msg: Joy):
        # Botones
        if self.rising_edge(msg.buttons, self.btn_run):
            self.send_cmd('run')

        if self.rising_edge(msg.buttons, self.btn_stop):
            self.send_cmd('stop')

        if self.rising_edge(msg.buttons, self.btn_reverse):
            self.send_cmd('reverse')

        if self.rising_edge(msg.buttons, self.btn_status):
            self.send_cmd('status')

        # Eje analógico para frecuencia
        if self.axis_speed < len(msg.axes):
            axis = msg.axes[self.axis_speed]

            if abs(axis) > self.deadzone:
                # arriba = subir, abajo = bajar
                self.target_hz += axis * self.step_hz
                self.target_hz = max(self.min_hz, min(self.max_hz, self.target_hz))
                self.send_cmd(f'freq {self.target_hz:.2f}')

        self.last_buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
