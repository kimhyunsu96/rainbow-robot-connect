#!/usr/bin/env python3
import socket, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RBIOBridge(Node):
    def __init__(self):
        super().__init__('rb_io_bridge')
        # params
        self.declare_parameter('host', '0.0.0.0')      # PC가 서버
        self.declare_parameter('port', 6000)
        self.declare_parameter('channel', 12)          # Din 채널 번호
        self.declare_parameter('reflect_topic', '/reflect_sensor')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = int(self.get_parameter('port').get_parameter_value().integer_value)
        self.channel = int(self.get_parameter('channel').get_parameter_value().integer_value)
        self.topic = self.get_parameter('reflect_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Bool, self.topic, 10)
        self.get_logger().info(f"Listening TCP {self.host}:{self.port}, ch={self.channel} -> {self.topic}")

        threading.Thread(target=self._server, daemon=True).start()

    def _server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            while rclpy.ok():
                self.get_logger().info("Waiting robot connection ...")
                conn, addr = s.accept()
                self.get_logger().info(f"Robot connected from {addr}")
                try:
                    self._handle(conn)
                except Exception as e:
                    self.get_logger().warn(f"Conn error: {e}")
                finally:
                    try: conn.close()
                    except: pass
                    self.get_logger().info("Robot disconnected")

    def _handle(self, conn: socket.socket):
        buf = b""
        conn.settimeout(1.0)
        while rclpy.ok():
            try:
                data = conn.recv(1024)
            except socket.timeout:
                continue
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = line.decode(errors='ignore').strip()
                # 기대형식: "DI <ch> <val>"  예) "DI 12 1"
                parts = msg.split()
                if len(parts) == 3 and parts[0] == 'DI':
                    try:
                        ch = int(parts[1]); val = int(parts[2])
                    except ValueError:
                        continue
                    if ch == self.channel and val in (0,1):
                        self.pub.publish(Bool(data=bool(val)))
                        self.get_logger().info(f"/reflect_sensor <- DI {ch} {val}")

def main():
    rclpy.init()
    node = RBIOBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
