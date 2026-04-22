#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
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


class L510Node(Node):
    def __init__(self):
        super().__init__('l510_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('slave', 1)
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('startup_hz', 10.0)
        self.declare_parameter('auto_run', False)
        self.declare_parameter('reverse', False)

        port = self.get_parameter('port').get_parameter_value().string_value
        slave = self.get_parameter('slave').get_parameter_value().integer_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        startup_hz = self.get_parameter('startup_hz').get_parameter_value().double_value
        auto_run = self.get_parameter('auto_run').get_parameter_value().bool_value
        reverse = self.get_parameter('reverse').get_parameter_value().bool_value

        self.ctrl = L510Controller(port=port, slave=slave, baudrate=baudrate)

        if not self.ctrl.connect():
            self.get_logger().error(f'No conecta al L510 en {port}')
            raise RuntimeError(f'No conecta al L510 en {port}')

        self.get_logger().info(f'Conectado al L510 en {port}, slave={slave}')

        if self.ctrl.set_frequency(startup_hz):
            self.get_logger().info(f'Frecuencia inicial: {startup_hz:.2f} Hz')
        else:
            self.get_logger().warning('No se pudo escribir frecuencia inicial')

        if auto_run:
            ok = self.ctrl.run_reverse() if reverse else self.ctrl.run_forward()
            if ok:
                self.get_logger().info(
                    'RUN reverse enviado' if reverse else 'RUN forward enviado'
                )
            else:
                self.get_logger().warning('No se pudo enviar RUN')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        st = self.ctrl.get_status()
        self.get_logger().info(
            f"state={st['state']} "
            f"err={st['err']} "
            f"freq_cmd={st['freq_cmd_hz']} Hz "
            f"freq_out={st['freq_out_hz']} Hz "
            f"current={st['current_raw']}"
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
    node = L510Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
