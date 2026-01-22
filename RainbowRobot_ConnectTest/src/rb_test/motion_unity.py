#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity 10Hz(string) → Robot bridge with built-in defaults

기본값(이전 CLI 인자와 동일)
- robot_ip=192.168.1.13
- unity_topic=/unity/pose_str
- step_dt=0.1, servo_Ts=0.008
- 위치 단위: unity_pos_unit=m, pos_scale=1000.0 (m→mm)
- 회전 단위: unity_rpy_unit=deg, rpy_in_deg=True
- 축 매핑: axis_map=[z,-x,y], rpy_map=[rz,rx,ry]
"""

import re
import time
import math
import threading
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Float32

from rb_test import cobot


def _as_float(x, default=0.0) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def _parse_values(s: str) -> List[float]:
    parts = [p.strip() for p in s.split(',')]
    return [_as_float(p) for p in parts if p != '']


def _map_axes(vec3: Tuple[float, float, float], axis_map: Tuple[str, str, str]) -> Tuple[float, float, float]:
    # axis_map 예: ("z", "-x", "y") → (xr, yr, zr) = (zu, -xu, yu)
    src = {'x': vec3[0], 'y': vec3[1], 'z': vec3[2]}
    out = []
    for m in axis_map:
        m = m.strip().lower()
        sign = -1.0 if m.startswith('-') else 1.0
        key = m.lstrip('-+')
        out.append(sign * src[key])
    return (out[0], out[1], out[2])


class UnityBridgeNode(Node):
    def __init__(self):
        super().__init__("motion_unity_bridge")

        # 기본값을 코드에 내장(요청 반영)
        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('unity_topic', '/unity/pose_str')
        self.declare_parameter('step_dt', 0.1)     # Unity 샘플 간격(초)
        self.declare_parameter('servo_Ts', 0.008)  # ServoJ 송신 간격(초)

        # 좌표계/단위 기본값
        self.declare_parameter('unity_pos_unit', 'm')   # 'm' or 'mm'
        self.declare_parameter('pos_scale', 1000.0)     # m→mm 배율
        self.declare_parameter('unity_rpy_unit', 'deg') # 'deg' or 'rad'
        self.declare_parameter('rpy_in_deg', True)

        # 축/회전 매핑 기본값
        self.declare_parameter('axis_map', ['z', '-x', 'y'])  # (Xr,Yr,Zr)=(Zu,-Xu,Yu)
        self.declare_parameter('rpy_map', ['rz', 'rx', 'ry']) # (rx,ry,rz)=(Rz,Rx,Ry)

        # ServoJ 파라미터(조인트 수신용)
        self.declare_parameter('servo_t1', 0.05)
        self.declare_parameter('servo_t2', 0.05)
        self.declare_parameter('servo_gain', 1.0)
        self.declare_parameter('servo_alpha', 0.003)
        self.declare_parameter('filter_alpha', 1.0)  # 1.0=LPF 끔

        # 실행 모드
        self.declare_parameter('send_mode', 'auto')  # 'auto'|'joint'|'cartesian'

        # 파라미터 읽기
        ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.unity_topic = self.get_parameter('unity_topic').get_parameter_value().string_value
        self.step_dt = float(self.get_parameter('step_dt').value)
        self.Ts = float(self.get_parameter('servo_Ts').value)

        self.unity_pos_unit = self.get_parameter('unity_pos_unit').get_parameter_value().string_value.lower()
        self.pos_scale = float(self.get_parameter('pos_scale').value)
        self.unity_rpy_unit = self.get_parameter('unity_rpy_unit').get_parameter_value().string_value.lower()
        self.rpy_in_deg = bool(self.get_parameter('rpy_in_deg').value)

        axis_map_list = list(self.get_parameter('axis_map').value)
        self.axis_map = (axis_map_list[0], axis_map_list[1], axis_map_list[2])
        rpy_map_list = list(self.get_parameter('rpy_map').value)
        self.rpy_map = (rpy_map_list[0], rpy_map_list[1], rpy_map_list[2])

        self.t1 = float(self.get_parameter('servo_t1').value)
        self.t2 = float(self.get_parameter('servo_t2').value)
        self.gain = float(self.get_parameter('servo_gain').value)
        self.alpha = float(self.get_parameter('servo_alpha').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)

        self.send_mode = self.get_parameter('send_mode').get_parameter_value().string_value.lower()

        # 상태/버퍼
        self.buf = deque(maxlen=30)  # 최근 3초 분량
        self.buf_lock = threading.Lock()
        self.current_joint = None  # type: Optional[List[float]]
        self.q_ref_prev = None     # ServoJ LPF 상태

        # ROS 통신
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=20, reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(String, self.unity_topic, self._on_str, qos)
        self.duration_pub = self.create_publisher(Float32, "/motion_executor/last_duration_s", 10)

        # 로봇 연결
        self._maybe_connect(ip)

        # 타이머/스레드
        self.step_timer = self.create_timer(self.step_dt, self._on_step)  # 10Hz 소비
        self.stream_thread = threading.Thread(target=self._stream_servoj_loop, daemon=True)
        self.stream_thread.start()

        self.get_logger().info(
            f"UnityBridge up: topic={self.unity_topic}, step_dt={self.step_dt:.3f}, Ts={self.Ts:.3f}, "
            f"axis_map={self.axis_map}, rpy_map={self.rpy_map}, pos_scale={self.pos_scale}"
        )

    def _maybe_connect(self, ip: str):
        try:
            if getattr(cobot, 'CMDSock', None) is None and hasattr(cobot, 'ConnectToCB'):
                self.get_logger().info(f"Connecting to robot controller at {ip} ...")
                cobot.ConnectToCB(ip)
        except Exception as e:
            self.get_logger().warn(f"ConnectToCB failed or not needed: {e}")

    # --- 수신/파싱 ---
    def _on_str(self, msg: String):
        text = msg.data.strip()
        # 허용 포맷: "<key>(val0, val1, ...)"
        m = re.match(r'^\s*([A-Za-z0-9_]+)\s*\(\s*([^\)]*?)\s*\)\s*$', text)
        if not m:
            return
        key = m.group(1).lower()
        vals = _parse_values(m.group(2))

        with self.buf_lock:
            self.buf.append((time.perf_counter(), key, vals))

    # --- 10Hz 스텝: 최신 샘플 소비 ---
    def _on_step(self):
        item = None
        with self.buf_lock:
            if self.buf:
                item = self.buf[-1]
                self.buf.clear()
        if not item:
            return

        _, key, vals = item

        # joint(...) → ServoJ 목표로 사용
        if (self.send_mode in ('auto', 'joint')) and key.startswith('joint') and len(vals) >= 6:
            q = [float(v) for v in vals[:6]]
            # 단위(deg/rad) 추정 보정: 매우 작은 값들이면 rad로 보고 deg로 변환
            if max(abs(v) for v in q) < 6.5:  # ~375deg < 6.5 rad
                q = [math.degrees(v) for v in q]
            self.current_joint = q
            return

        # link/tcp (...) → 좌표계 변환 후 MovePB 한 점 실행(간단)
        if (self.send_mode in ('auto', 'cartesian')) and len(vals) >= 6:
            pos_u = (vals[0], vals[1], vals[2])
            rpy_u = (vals[3], vals[4], vals[5])

            # 위치 단위 변환
            scale = self.pos_scale if self.unity_pos_unit == 'm' else 1.0
            pos_u = (pos_u[0]*scale, pos_u[1]*scale, pos_u[2]*scale)

            # 회전 단위 변환(rad→deg 필요 시)
            if self.unity_rpy_unit == 'rad' and self.rpy_in_deg:
                rpy_u = tuple(math.degrees(v) for v in rpy_u)

            # 축 매핑
            pos_r = _map_axes(pos_u, self.axis_map)

            # rpy 매핑: 'rx','ry','rz' 키 선택
            src_r = {'rx': rpy_u[0], 'ry': rpy_u[1], 'rz': rpy_u[2]}
            rx = src_r[self.rpy_map[0].lower()]
            ry = src_r[self.rpy_map[1].lower()]
            rz = src_r[self.rpy_map[2].lower()]

            # 필요 시 오프셋/보정(툴/TCP, 베이스) 추가 가능
            try:
                p = cobot.Point(float(pos_r[0]), float(pos_r[1]), float(pos_r[2]),
                                float(rx), float(ry), float(rz))
                # 간단 구현: 스텝마다 1점만 큐잉하고 실행
                try:
                    if hasattr(cobot, 'MovePB_Clear'):
                        cobot.MovePB_Clear()
                except Exception:
                    pass
                # 속도/블렌드는 환경에 맞게 조정
                try:
                    cobot.MovePB_Add(p, 100.0, cobot.BLEND_OPTION.RATIO, 0.0)
                except Exception:
                    cobot.MovePB_Add(p, 100.0, 0, 0.0)
                try:
                    cobot.MovePB_Run(100.0, cobot.BLEND_RTYPE.INTENDED)
                except Exception:
                    try:
                        cobot.MovePB_Run(100.0)
                    except Exception:
                        cobot.MovePB_Run()
            except Exception as e:
                self.get_logger().warn(f"MovePB step failed: {e}")
            return

        # 알 수 없는 키 → 무시
        self.get_logger().debug(f"Unknown key or values: {key}({len(vals)})")

    # --- ServoJ 스트리밍(조인트 스텝-홀드) ---
    def _stream_servoj_loop(self):
        Ts = self.Ts
        t1, t2, gain, alpha = self.t1, self.t2, self.gain, self.alpha
        filt = max(0.0, min(1.0, self.filter_alpha))

        t0 = time.perf_counter()
        k = 0
        start_perf = time.perf_counter()

        while rclpy.ok():
            target = self.current_joint
            if target is not None:
                if self.q_ref_prev is None:
                    self.q_ref_prev = list(target)
                # 외부 LPF
                if 0.0 < filt < 1.0:
                    q_ref = [(1.0 - filt) * self.q_ref_prev[i] + filt * target[i] for i in range(6)]
                else:
                    q_ref = list(target)
                try:
                    j = cobot.Joint(*q_ref)
                    cobot.ServoJ(j.j0, j.j1, j.j2, j.j3, j.j4, j.j5, t1, t2, gain, alpha)
                except Exception:
                    pass
                self.q_ref_prev = q_ref

            # 타이밍 정렬
            t_next = t0 + k * Ts
            now = time.perf_counter()
            remain = t_next - now
            if remain > 0.0015:
                time.sleep(remain - 0.0007)
            while time.perf_counter() < t_next:
                pass
            k += 1

        try:
            elapsed = time.perf_counter() - start_perf
            self.duration_pub.publish(Float32(data=float(elapsed)))
        except Exception:
            pass


def main():
    rclpy.init()
    node = UnityBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()