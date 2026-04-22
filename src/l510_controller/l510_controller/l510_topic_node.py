#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymodbus.client import ModbusSerialClient


REG_OP_SIGNAL = 9473   # 0x2501
REG_FREQ_CMD  = 9474   # 0x2502

REG_STATE     = 9504   # 0x2520
REG_ERR       = 9505   # 0x2521
REG_FREQ_RD   = 9507   # 0x2523
REG_FREQ_OUT  = 9508   # 0x2524
REG_CURRENT   = 9511   # 0x2527

MIN_HZ = 0.0
MAX_HZ = 60.0
STEP_HZ = 1.0


class L510Controller:
    def __init__(self, port: str, slave: int, baudrate: int = 9600):
        self.slave = slave
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=1.0,
        )
        self.last_freq_hz = 10.0

    def connect(self) -> bool:
        return self.client.connect()

    def close(self):
        self.client.close()

    def read1(self, addr: int):
        rr = self.client.read_holding_registers(addr, 1, slave=self.slave)
        if rr.isError():
            return None
        return rr.registers[0]

    def write1(self, addr: int, value: int) -> bool:
        wr = self.client.write_register(addr, value, slave=self.slave)
        return not wr.isError()

    def hz_to_word(self, hz: float) -> int:
        hz = max(MIN_HZ, min(MAX_HZ, hz))
        return int(round(hz * 100))

    def word_to_hz(self, word):
        if word is None:
            return None
        return word / 100.0

    def set_frequency(self, hz: float) -> bool:
        hz = max(MIN_HZ, min(MAX_HZ, hz))
        ok = self.write1(REG_FREQ_CMD, self.hz_to_word(hz))
        if ok:
            self.last_freq_hz = hz
        return ok

    def run_forward(self) -> bool:
        return self.write1(REG_OP_SIGNAL, 1)

    def run_reverse(self) -> bool:
        return self.write1(REG_OP_SIGNAL, 3)

    def stop(self) -> bool:
        return self.write1(REG_OP_SIGNAL, 0)

    def get_status(self) -> dict:
        state = self.read1(REG_STATE)
        err = self.read1(REG_ERR)
        fcmd = self.read1(REG_FREQ_RD)
        fout = self.read1(REG_FREQ_OUT)
        curr = self.read1(REG_CURRENT)
        return {
            "state": state,
            "err": err,
            "freq_cmd_word": fcmd,
            "freq_out_word": fout,
            "freq_cmd_hz": self.word_to_hz(fcmd),
            "freq_out_hz": self.word_to_hz(fout),
            "current_raw": curr,
        }


class L510TopicNode(Node):
    def __init__(self):
        super().__init__('l510_topic_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('slave', 1)
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').value
        slave = self.get_parameter('slave').value
        baudrate = self.get_parameter('baudrate').value

        self.ctrl = L510Controller(port=port, slave=slave, baudrate=baudrate)

        if not self.ctrl.connect():
            self.get_logger().error(f'No conecta al L510 en {port}')
            raise RuntimeError(f'No conecta al L510 en {port}')

        self.get_logger().info(f'Conectado al L510 en {port}, slave={slave}')

        self.subscription = self.create_subscription(
            String,
            '/l510_cmd',
            self.cmd_callback,
            10
        )

        self.status_timer = self.create_timer(1.0, self.print_status)

        self.presets = {
            '1': 10.0,
            '2': 20.0,
            '3': 30.0,
            '4': 40.0,
            '5': 50.0,
            '6': 60.0,
        }

    def print_status(self):
        st = self.ctrl.get_status()
        self.get_logger().info(
            f"state={st['state']}  "
            f"err={st['err']}  "
            f"freq_cmd={st['freq_cmd_hz']} Hz  "
            f"freq_out={st['freq_out_hz']} Hz  "
            f"current={st['current_raw']}"
        )

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Comando recibido: '{cmd}'")

        if cmd == 'run':
            if self.ctrl.run_forward():
                self.get_logger().info('RUN forward enviado.')
            else:
                self.get_logger().error('Falló RUN forward.')
            time.sleep(0.3)
            self.print_status()
            return

        if cmd == 'reverse':
            if self.ctrl.run_reverse():
                self.get_logger().info('RUN reverse enviado.')
            else:
                self.get_logger().error('Falló RUN reverse.')
            time.sleep(0.3)
            self.print_status()
            return

        if cmd == 'stop':
            if self.ctrl.stop():
                self.get_logger().info('STOP enviado.')
            else:
                self.get_logger().error('Falló STOP.')
            time.sleep(0.3)
            self.print_status()
            return

        if cmd == 'inc' or cmd == '+':
            new_hz = min(MAX_HZ, self.ctrl.last_freq_hz + STEP_HZ)
            if self.ctrl.set_frequency(new_hz):
                self.get_logger().info(f'Frecuencia -> {new_hz:.2f} Hz')
            else:
                self.get_logger().error('Falló escritura de frecuencia.')
            time.sleep(0.2)
            self.print_status()
            return

        if cmd == 'dec' or cmd == '-':
            new_hz = max(MIN_HZ, self.ctrl.last_freq_hz - STEP_HZ)
            if self.ctrl.set_frequency(new_hz):
                self.get_logger().info(f'Frecuencia -> {new_hz:.2f} Hz')
            else:
                self.get_logger().error('Falló escritura de frecuencia.')
            time.sleep(0.2)
            self.print_status()
            return

        if cmd in self.presets:
            hz = self.presets[cmd]
            if self.ctrl.set_frequency(hz):
                self.get_logger().info(f'Frecuencia preset -> {hz:.2f} Hz')
            else:
                self.get_logger().error('Falló escritura de frecuencia.')
            time.sleep(0.2)
            self.print_status()
            return

        if cmd == 'status':
            self.print_status()
            return

        if cmd.startswith('freq '):
            raw = cmd.split(' ', 1)[1].strip()
            try:
                hz = float(raw)
            except ValueError:
                self.get_logger().error(f'Valor inválido para frecuencia: {raw}')
                return

            if self.ctrl.set_frequency(hz):
                hz_clamped = max(MIN_HZ, min(MAX_HZ, hz))
                self.get_logger().info(f'Frecuencia -> {hz_clamped:.2f} Hz')
            else:
                self.get_logger().error('Falló escritura de frecuencia.')
            time.sleep(0.2)
            self.print_status()
            return

        self.get_logger().warning(
            "Comando no reconocido. Usa: run, reverse, stop, inc, dec, "
            "1..6, status, freq <hz>"
        )

    def destroy_node(self):
        try:
            self.ctrl.stop()
            time.sleep(0.2)
        except Exception:
            pass
        try:
            self.ctrl.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = L510TopicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
