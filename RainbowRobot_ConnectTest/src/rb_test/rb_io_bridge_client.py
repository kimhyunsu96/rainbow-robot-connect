#!/usr/bin/env python3
import socket, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RBIOBridgeClient(Node):
    def __init__(self):
        super().__init__('rb_io_bridge_client')
        # params
        self.declare_parameter('robot_ip', '192.168.1.13')  # 로봇 컨트롤러 IP
        self.declare_parameter('robot_port', 6000)          # 로봇 소켓 서버 포트
        self.declare_parameter('channel', 12)               # 관심 Din 채널
        self.declare_parameter('reflect_topic', '/reflect_sensor')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = int(self.get_parameter('robot_port').get_parameter_value().integer_value)
        self.channel = int(self.get_parameter('channel').get_parameter_value().integer_value)
        self.topic = self.get_parameter('reflect_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(Bool, self.topic, 10)
        self.get_logger().info(f"Connecting to robot {self.robot_ip}:{self.robot_port}, ch={self.channel} -> {self.topic}")
        self.create_timer(0.5, self._ensure_conn)
        self.conn = None
        self.buf = b''

    def _ensure_conn(self):
        if self.conn:  # 이미 연결됨 → 수신 처리
            try:
                self.conn.settimeout(0.0)
                data = self.conn.recv(4096)
            except BlockingIOError:
                return
            except Exception as e:
                self.get_logger().warn(f"recv error: {e}")
                self._close(); return
            if not data:
                self.get_logger().info("robot closed"); self._close(); return
            self.buf += data
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                msg = line.decode(errors='ignore').strip()
                parts = msg.split()
                if len(parts) == 3 and parts[0] == 'DI':
                    try:
                        ch = int(parts[1]); val = int(parts[2])
                    except ValueError:
                        continue
                    if ch == self.channel and val in (0,1):
                        self.pub.publish(Bool(data=bool(val)))
                        self.get_logger().info(f"/reflect_sensor <- DI {ch} {val}")
        else:
            # 연결 시도
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self.robot_ip, self.robot_port))
                s.settimeout(None)
                self.conn = s
                self.get_logger().info("Connected to robot")
            except Exception as e:
                self.get_logger().info(f"connect retry: {e}")
                time.sleep(1.0)

    def _close(self):
        try:
            if self.conn:
                self.conn.close()
        finally:
            self.conn = None

def main():
    rclpy.init()
    node = RBIOBridgeClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
