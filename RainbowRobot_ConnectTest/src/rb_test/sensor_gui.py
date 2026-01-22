#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt


class SensorSub(Node):
    """ROS2 subscriber node that listens to /reflect_sensor (std_msgs/Bool)."""
    def __init__(self, topic_name: str = '/reflect_sensor'):
        super().__init__('sensor_gui_sub')
        self.state = 0  # 0/1
        self.sub = self.create_subscription(Bool, topic_name, self._callback, 10)

    def _callback(self, msg: Bool):
        self.state = 1 if msg.data else 0


class SensorGUI(QWidget):
    """Simple PyQt5 GUI to display the sensor's 0/1 state."""
    def __init__(self, node: SensorSub, executor: SingleThreadedExecutor):
        super().__init__()
        self.node = node
        self.executor = executor

        self.setWindowTitle('Rainbow Sensor Monitor')
        self.resize(360, 200)

        self.label = QLabel('0')
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet(self._style_for(0))

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Spin ROS in a Qt timer and refresh UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)  # ms

    def _style_for(self, v: int) -> str:
        # 1: green, 0: gray
        if v == 1:
            return ("font-size:72pt; font-weight:bold; color:white; "
                    "background:#2aa745; border-radius:16px;")
        else:
            return ("font-size:72pt; font-weight:bold; color:#333; "
                    "background:#d0d0d0; border-radius:16px;")

    def _tick(self):
        # Process ROS callbacks
        self.executor.spin_once(timeout_sec=0.0)
        # Update label
        v = self.node.state
        self.label.setText('1' if v == 1 else '0')
        self.label.setStyleSheet(self._style_for(v))


def main():
    rclpy.init()
    node = SensorSub('/reflect_sensor')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    app = QApplication(sys.argv)
    w = SensorGUI(node, executor)
    w.show()

    code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
