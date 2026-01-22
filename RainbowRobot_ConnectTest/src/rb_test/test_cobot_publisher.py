#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
from typing import List

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QFileDialog, QVBoxLayout, QLabel

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rb_test.motion_executor import MotionExecutor
from rb_test import cobot

DEFAULT_ROBOT_IP = '192.168.1.13'


class MotionFileLoader(QWidget):
    def __init__(self, executor: MotionExecutor):
        super().__init__()
        self.executor = executor
        self.setWindowTitle("Motion Loader")
        self.resize(380, 140)
        self.label = QLabel("📁 파일을 선택하세요")
        self.btn   = QPushButton("🔍 Load Motion File")
        self.btn.clicked.connect(self.load_file)
        lay = QVBoxLayout(self)
        lay.addWidget(self.label); lay.addWidget(self.btn)

    def load_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Motion File", "", "YAML or JSON (*.yaml *.yml *.json)"
        )
        if not path:
            return
        self.label.setText(f"📄 {os.path.basename(path)} 선택됨")
        self.executor.load_motion_from_file(path)


class HeadlessLauncher(Node):
    def __init__(self):
        super().__init__('headless_motion_launcher')

        # ---- Parameters ----
        self.declare_parameter('robot_ip', DEFAULT_ROBOT_IP)
        self.declare_parameter('motion_file', '')
        self.declare_parameter('auto_load', False)

        # one-pose direct params
        self.declare_parameter(
            'pose_arr',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter('move', 'lin')      # lin | joint
        self.declare_parameter('coord', 'base')    # base | user | local
        self.declare_parameter('speed', 30.0)
        self.declare_parameter('accel', 30.0)

        robot_ip   = self.get_parameter('robot_ip').value
        motion_file = self.get_parameter('motion_file').value
        auto_load   = bool(self.get_parameter('auto_load').value)

        pose_arr = list(self.get_parameter('pose_arr').value)
        move_type = str(self.get_parameter('move').value).lower()
        coord     = str(self.get_parameter('coord').value).lower()
        speed     = float(self.get_parameter('speed').value)
        accel     = float(self.get_parameter('accel').value)

        # ---- Robot connect & executor ----
        cobot.ConnectToCB(robot_ip)
        self.motion_exec = MotionExecutor()
        self.get_logger().info("✅ MotionExecutor 노드 초기화 완료")

        # ---- direct one-pose? ----
        pose_arr_set = any(abs(x) > 1e-12 for x in pose_arr)
        if pose_arr_set:
            pose_list = [float(x) for x in pose_arr[:6]]
            self.get_logger().info(
                f"[ONE-POSE] move={move_type}, coord={coord}, speed={speed}, accel={accel} → {pose_list}"
            )

            ran_direct = False
            try:
                if hasattr(self.motion_exec, "run_single_pose"):
                    self.motion_exec.run_single_pose(
                        pose_list, move=move_type, coord=coord, speed=speed, accel=accel
                    )
                    ran_direct = True
                elif hasattr(self.motion_exec, "add_pose") and hasattr(self.motion_exec, "execute"):
                    self.motion_exec.add_pose(
                        pose_list, move=move_type, coord=coord, speed=speed, accel=accel
                    )
                    self.motion_exec.execute()
                    ran_direct = True
            except Exception as e:
                self.get_logger().warn(f"direct API 실행 실패: {e}")

            if not ran_direct:
                try:
                    if hasattr(cobot, "MovePB_Clear") and hasattr(cobot, "MovePB_Add") and hasattr(cobot, "MovePB_Start"):
                        cobot.MovePB_Clear()
                        try:
                            cobot.MovePB_Add(pose_list)  # (x,y,z,rx,ry,rz)
                        except TypeError:
                            cobot.MovePB_Add(pose_list, move_type, coord, speed, accel)
                        cobot.MovePB_Start()
                        ran_direct = True
                        self.get_logger().info("[ONE-POSE] cobot direct OK")
                except Exception as e:
                    self.get_logger().error(f"[ONE-POSE] cobot direct 실패: {e}")

            # end process (항상 즉시 종료해 GUI가 다음 트리거를 받을 수 있게)
            time.sleep(0.1)
            try:
                rclpy.shutdown()
            finally:
                os._exit(0)

        # ---- motion_file + auto_load? ----
        if motion_file and os.path.exists(motion_file) and auto_load:
            self.get_logger().info(f"[AUTO] run: {motion_file}")
            self.motion_exec.load_motion_from_file(motion_file)
            time.sleep(0.1)
            try:
                rclpy.shutdown()
            finally:
                os._exit(0)

        # ---- else: manual loader GUI ----
        app = QApplication(sys.argv)
        w = MotionFileLoader(self.motion_exec)
        w.show()
        app.exec_()
        rclpy.shutdown()


def main():
    rclpy.init()
    _ = HeadlessLauncher()
    # no spin; node exits after headless run or GUI closed


if __name__ == '__main__':
    main()
