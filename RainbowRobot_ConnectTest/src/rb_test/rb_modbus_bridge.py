#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# ── pymodbus 2.x / 3.x 호환 import ──
try:
    from pymodbus.client import ModbusTcpClient  # 3.x
    PM3 = True
except Exception:
    from pymodbus.client.sync import ModbusTcpClient  # 2.x (2.5.3)
    PM3 = False

from pymodbus.exceptions import ModbusIOException


class RBModbusBridge(Node):
    """
    Modbus/TCP로 DI(또는 Coil)을 폴링해서 /reflect_sensor(Bool) 퍼블리시.
    - robot_ip: 컨트롤러 IP
    - port: 502
    - di_address: 0-based 주소
    - function: 'di' 또는 'coil' (장비 매핑에 따라 선택)
    - slave_id: 1(기본). 장비에 따라 0/255 등을 시도하세요.
    """
    def __init__(self):
        super().__init__('rb_modbus_bridge')

        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('port', 502)
        self.declare_parameter('di_address', 12)
        self.declare_parameter('poll_ms', 50)
        self.declare_parameter('invert', False)
        self.declare_parameter('function', 'di')   # 'di' or 'coil'
        self.declare_parameter('slave_id', 1)

        self.robot_ip   = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.port       = int(self.get_parameter('port').get_parameter_value().integer_value)
        self.di_address = int(self.get_parameter('di_address').get_parameter_value().integer_value)
        self.poll_ms    = int(self.get_parameter('poll_ms').get_parameter_value().integer_value)
        self.invert     = bool(self.get_parameter('invert').get_parameter_value().bool_value)
        self.function   = self.get_parameter('function').get_parameter_value().string_value
        self.slave_id   = int(self.get_parameter('slave_id').get_parameter_value().integer_value)

        self.pub = self.create_publisher(Bool, '/reflect_sensor', 10)

        self.client: Optional[ModbusTcpClient] = None
        self.last_state: Optional[int] = None

        self._connect()
        self.timer = self.create_timer(self.poll_ms / 1000.0, self._poll)

        self.get_logger().info(
            f"Modbus DI reader → {self.robot_ip}:{self.port}, addr={self.di_address}, "
            f"func={self.function}, poll={self.poll_ms}ms, invert={self.invert}, "
            f"slave={self.slave_id}, pymodbus3={PM3}"
        )

    # ── 내부 함수들 ──────────────────────────────────────────────
    def _connect(self):
        try:
            if self.client:
                self.client.close()
        except Exception:
            pass
        self.client = ModbusTcpClient(self.robot_ip, port=self.port)
        ok = self.client.connect()
        self.get_logger().info(f"Modbus connect({self.robot_ip}:{self.port}) -> {ok}")

    def _is_connected(self) -> bool:
        if not self.client:
            return False
        if PM3:
            # pymodbus 3.x
            return bool(getattr(self.client, 'connected', False))
        else:
            # pymodbus 2.5.3
            try:
                return bool(self.client.is_socket_open())
            except Exception:
                return False

    def _read_bit(self):
        if not self.client or not self._is_connected():
            return None

        addr = self.di_address
        if self.function.lower() == 'coil':
            rr = (self.client.read_coils(addr, 1, slave=self.slave_id) if PM3
                  else self.client.read_coils(addr, 1, unit=self.slave_id))
        else:
            rr = (self.client.read_discrete_inputs(addr, 1, slave=self.slave_id) if PM3
                  else self.client.read_discrete_inputs(addr, 1, unit=self.slave_id))

        if hasattr(rr, "isError") and rr.isError():
            from pymodbus.exceptions import ModbusIOException
            raise ModbusIOException(str(rr))
        if not hasattr(rr, "bits") or not rr.bits:
            raise Exception("no bits")
        return 1 if rr.bits[0] else 0

    def _poll(self):
        if not self.client or not self._is_connected():
            self._connect()
            return
        try:
            bit = self._read_bit()
            if bit is None:
                return
            if self.invert:
                bit = 0 if bit == 1 else 1
            if self.last_state is None or bit != self.last_state:
                self.last_state = bit
                self.pub.publish(Bool(data=bool(bit)))
                self.get_logger().info(f"/reflect_sensor = {bit}")
        except Exception as e:
            self.get_logger().warn(f"Modbus read error: {e}")
            time.sleep(0.2)
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None

    def destroy_node(self):
        try:
            if self.client:
                self.client.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = RBModbusBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
