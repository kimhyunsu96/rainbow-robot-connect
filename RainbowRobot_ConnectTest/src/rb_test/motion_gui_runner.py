#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from pathlib import Path
import json
import time
import subprocess
import threading
from collections import deque
from copy import deepcopy
import numpy as np

from rb_test.motion_executor import MotionExecutor
from rb_test import cobot
from statistics import median

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QFileDialog, QSpinBox,
    QDoubleSpinBox, QGroupBox, QGridLayout, QComboBox, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer, Qt, QObject, pyqtSignal
from PyQt5.QtGui import QFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
)
from rclpy.duration import Duration
import yaml
import tempfile
from std_msgs.msg import Float32

import math


# Unity 실시간 브리지 통합
from rb_test.unity_tcp_bridge import UnityTcpBridge
import asyncio

class AutoModeController(QObject):
    status_signal = pyqtSignal(str)    # GUI 하단 상태줄 업데이트용
    running_signal = pyqtSignal(bool)  # Start/Stop 버튼 토글용

    def __init__(self, motion_executor_fn, sensor_state_fn,
                 edge_events_fn=None, is_busy_fn=None, parent=None):
        """
        motion_executor_fn: callable(motion_file, chunk_size, chunk_delay_ms) -> bool
        sensor_state_fn   : callable() -> int  # 최신 센서값(0/1) 반환
        edge_events_fn    : callable() -> list[float]  # 실센서 상승엣지 타임스탬프 꺼내오기
        is_busy_fn        : callable() -> bool # 실행기 busy 여부(실제 모션 종료 감지)
        """
        super().__init__(parent)
        self.motion_executor_fn = motion_executor_fn
        self.sensor_state_fn = sensor_state_fn
        self._edge_events_fn = edge_events_fn
        self._is_busy_fn = is_busy_fn or (lambda: False)

        self.timer = QTimer(self)
        self.timer.setInterval(100)    # 10Hz 루프
        self.timer.timeout.connect(self._loop)

        self._running = False
        self._preset = None
        self._next_phase = "idle"
        self._phase_until = 0.0
        self._trigger_mode = "schedule"
        self._prev_sensor = 0

        # 타임스탬프 큐(대기/모션 중 들어온 감지 저장)
        self._edge_queue = deque()
        # 간격 필터 기준(허용된 마지막 엣지 시각)
        self._last_accepted_edge_time = None
        # 자동모드 시작 시각(ON 이전 엣지 무시용)
        self._auto_started_at = 0.0

        # 마지막 값만 적용하기 위한 보류 값(큐 사용 안 함)
        self._has_pending_value = False
        self._pending_value = None  # int 또는 None(스킵)

    def is_running(self) -> bool:
        return self._running

    def load_preset(self, preset: dict):
        self._preset = preset
        self._trigger_mode = preset.get("trigger_mode", "schedule")
        # 값→모션 파일 매핑 로드
        self._value_map = {}
        vm = preset.get("value_map") or {}
        # 키를 정수로 정규화
        try:
            for k, v in vm.items():
                self._value_map[int(k)] = str(v)
        except Exception:
            self._value_map = {}

    def start(self):
        if not self._preset:
            self.status_signal.emit("자동모드: 프리셋이 선택되지 않았습니다.")
            return
        self._running = True
        self.running_signal.emit(True)
        self.status_signal.emit(f"자동모드 시작: {self._preset['name']} (mode={self._trigger_mode})")

        if self._trigger_mode == "schedule":
            self._schedule_phase("wait_before", self._preset.get("wait_before_s", 0.0))
        elif self._trigger_mode == "sensor":
            self._next_phase = "arm_edge"
            self._phase_until = 0.0
            try:
                self.status_signal.emit("센서 트리거 대기 중...")
            except Exception:
                pass
        elif self._trigger_mode == "tcp_value":
            # TCP 값 대기 모드
            self._next_phase = "wait_value"
            self._phase_until = 0.0
            self.status_signal.emit("TCP 값 대기 중...")
        else:
            self._schedule_phase("wait_before", self._preset.get("wait_before_s", 0.0))

        # 시작 시 큐/기준시간 초기화 + 시작 시각 기록
        self._edge_queue.clear()
        self._last_accepted_edge_time = None
        self._auto_started_at = time.time()
        self.timer.start()

    def feed_value(self, v):
        """Deprecated: 큐 대신 마지막 값만 보류. 정수 또는 None(스킵) 허용."""
        self.set_pending_value(v)

    def set_pending_value(self, v):
        """연속으로 들어오는 값에서 마지막 것만 보관. int 또는 None(스킵)."""
        try:
            if isinstance(v, str) and v.strip().lower() == 'none':
                self._pending_value = None
                self._has_pending_value = True
            elif v is None:
                self._pending_value = None
                self._has_pending_value = True
            else:
                self._pending_value = int(v)
                self._has_pending_value = True
        except Exception:
            pass

    def stop(self):
        self._running = False
        self.running_signal.emit(False)
        self.timer.stop()
        self._next_phase = "idle"
        self.status_signal.emit("자동모드 중지")
        self._edge_queue.clear()
        self._last_accepted_edge_time = None
        self._auto_started_at = 0.0

    def _schedule_phase(self, phase: str, duration_s: float):
        self._next_phase = phase
        self._phase_until = time.time() + max(0.0, float(duration_s or 0.0))

    def _schedule_wait_from_edge(self, edge_time: float):
        """edge_time 기준으로 남은 대기시간을 계산해 wait_before 단계로 진입"""
        wb = float(self._preset.get("wait_before_s", 0.0) or 0.0)
        now = time.time()
        remain = wb - (now - float(edge_time))
        if remain < 0:
            # 음수면 즉시 실행처럼 보이므로 0으로 보정
            self.status_signal.emit(f"[잔여대기 보정] remain={remain:.2f}s → 0.00s")
            remain = 0.0
        self._schedule_phase("wait_before", remain)

    def _loop(self):
        if not self._running or not self._preset:
            return

        now = time.time()
        try:
            sensor_val = int(self.sensor_state_fn())
        except Exception:
            sensor_val = 0

        # -------- tcp_value trigger mode (센서 엣지 기반, 값 큐 매칭) --------
        if self._trigger_mode == "tcp_value":
            # 센서 엣지를 가져오기(노드 제공 콜백 활용)
            edges = []
            if self._edge_events_fn is not None:
                edges = self._edge_events_fn()  # list[float]
            else:
                # 콜백이 없으면 간단히 현재 샘플로 엣지 검출
                edge_rise = (self._prev_sensor == 0 and sensor_val == 1)
                if edge_rise:
                    edges = [now]

            # 자동모드 ON 이전 타임스탬프는 폐기
            if self._auto_started_at:
                edges = [t for t in edges if t >= self._auto_started_at]

            # 간격 필터 (센서 모드와 동일 정책 적용)
            exp_iv = float(self._preset.get("expected_interval_s", 0.0) or 0.0)
            min_gap = max(0.0, exp_iv - 2.0)
            if self._last_accepted_edge_time is None:
                filtered = edges[:]
            else:
                filtered = []
                for t in edges:
                    if (t - self._last_accepted_edge_time) >= min_gap:
                        filtered.append(t)
                    else:
                        self.status_signal.emit(
                            f"엣지 무시: 간격 {(t - self._last_accepted_edge_time):.1f}s < 허용 {min_gap:.1f}s"
                        )
            edges = filtered

            phase_before = self._next_phase

            if self._next_phase in ("wait_value", "arm_edge"):
                # 센서 엣지가 오면 그 순간 '해당 사이클의 값'을 큐에서 꺼냄
                if edges:
                    t0 = edges.pop(0)
                    self._last_accepted_edge_time = t0

                    # 마지막 보류값만 사용(큐 사용 안 함)
                    val = None
                    if getattr(self, '_has_pending_value', False):
                        val = self._pending_value
                        self._pending_value = None
                        self._has_pending_value = False

                    if val is None:
                        # none 또는 미수신 → 사이클 건너뜀
                        self.status_signal.emit("센서 감지: 최근값 없음/none → 사이클 건너뜀")
                        self._next_phase = "arm_edge"
                        self._phase_until = 0.0
                    else:
                        # 값→모션 파일 매핑 확인
                        motion_path = self._value_map.get(int(val))
                        if motion_path and os.path.exists(motion_path):
                            self._pending_motion_override = motion_path
                            self.status_signal.emit(f"센서 감지: 값 {val} → {os.path.basename(motion_path)} 예약")
                            # 센서 시각 기준으로 wait_before 보정
                            self._schedule_wait_from_edge(t0)
                            # 다음 단계로 이동은 _schedule_wait_from_edge에서 처리됨
                        else:
                            self.status_signal.emit(f"센서 감지: 값 {val} 매핑 없음/파일 없음 → 사이클 건너뜀")
                            self._next_phase = "arm_edge"
                            self._phase_until = 0.0

            elif self._next_phase == "wait_before":
                if now >= self._phase_until:
                    # 실행
                    mf = getattr(self, "_pending_motion_override", None) or self._preset.get("motion_file", "")
                    chunk_size = int(self._preset.get("chunk_size", 0) or 0)
                    chunk_delay_ms = int(self._preset.get("chunk_delay_ms", 0) or 0)
                    ok = self.motion_executor_fn(mf, chunk_size, chunk_delay_ms)
                    if ok:
                        dur = float(self._preset.get("motion_duration_s", 0.0) or 0.0)
                        # 실제 완료를 기다리고 싶으면 is_busy_fn을 활용해 0으로 설정하고 폴링
                        self._schedule_phase("wait_motion_done", dur)
                        self.status_signal.emit(f"모션 실행: {os.path.basename(mf)}")
                    else:
                        self.status_signal.emit("모션 실행 실패 → 다음 센서 대기")
                        self._next_phase = "arm_edge"
                        self._phase_until = 0.0
                    self._pending_motion_override = None

            elif self._next_phase == "wait_motion_done":
                # 모션 완료 후 다음 사이클 대기
                if now >= self._phase_until or not self._is_busy_fn():
                    self._next_phase = "arm_edge"
                    self._phase_until = 0.0
                    self.status_signal.emit("모션 완료 → 다음 센서 대기")

            # 대기/모션 중 들어온 남은 센서 엣지들은 큐에 적재 (연속 처리 지원)
            if phase_before in ("wait_before", "wait_motion_done") and edges:
                max_q = int(self._preset.get("max_queue", 0) or 0)
                for t in edges:
                    if max_q > 0 and len(self._edge_queue) >= max_q:
                        break
                    self._edge_queue.append(t)
                    self._last_accepted_edge_time = t
                self.status_signal.emit(f"엣지 적재(대기열={len(self._edge_queue)})")

            self._prev_sensor = sensor_val
            remain = max(0.0, self._phase_until - now)
            has_pend = getattr(self, '_has_pending_value', False)
            pdisp = f"Y(v={self._pending_value})" if has_pend else "N"
            self.status_signal.emit(
                f"[TCP+센서:{sensor_val}] 단계:{self._next_phase} 남은:{remain:0.1f}s, pending:{pdisp}"
            )
            return

        # -------- sensor trigger mode --------
        if self._trigger_mode == "sensor":
            now_sec = time.time()

            # 콜백 큐에서 엣지 타임스탬프 수집 (없으면 샘플링 fallback)
            edges = []
            if self._edge_events_fn is not None:
                edges = self._edge_events_fn()  # list[float]
            else:
                edge_rise = (self._prev_sensor == 0 and sensor_val == 1)
                if edge_rise:
                    edges = [now_sec]

            # 자동모드 ON 이전 타임스탬프는 폐기
            if self._auto_started_at:
                edges = [t for t in edges if t >= self._auto_started_at]

            # 간격 필터: expected_interval_s - 2.0 미만은 무시 (첫 엣지는 허용)
            exp_iv = float(self._preset.get("expected_interval_s", 0.0) or 0.0)
            min_gap = max(0.0, exp_iv - 2.0)
            if self._last_accepted_edge_time is None:
                filtered = edges[:]
            else:
                filtered = []
                for t in edges:
                    if (t - self._last_accepted_edge_time) >= min_gap:
                        filtered.append(t)
                    else:
                        self.status_signal.emit(
                            f"엣지 무시: 간격 {(t - self._last_accepted_edge_time):.1f}s < 허용 {min_gap:.1f}s"
                        )
            edges = filtered

            phase_before = self._next_phase

            if self._next_phase == "arm_edge":
                if self._edge_queue:
                    t0 = self._edge_queue.popleft()
                    self.status_signal.emit("대기열 소모 → 대기 시작(잔여)")
                    self._schedule_wait_from_edge(t0)
                elif edges:
                    t0 = edges.pop(0)
                    self._last_accepted_edge_time = t0
                    self.status_signal.emit("엣지 감지 → 대기 시작")
                    self._schedule_wait_from_edge(t0)

            elif now >= self._phase_until:
                if self._next_phase == "wait_before":
                    mf = self._preset["motion_file"]
                    cs = int(self._preset.get("chunk_size", 0) or 0)
                    cd = int(self._preset.get("chunk_delay_ms", 0) or 0)
                    ok = self.motion_executor_fn(mf, cs, cd)
                    if ok:
                        # 모션 시간에 의존하지 않고 실제 완료를 기다림
                        self._schedule_phase("wait_motion_done", 0.0)
                    else:
                        self._schedule_phase("arm_edge", 0.0)

                elif self._next_phase == "wait_motion_done":
                    # 실행기가 바쁘지 않으면(=모션 끝) 다음 사이클로
                    if not self._is_busy_fn():
                        if self._edge_queue:
                            t0 = self._edge_queue.popleft()
                            self.status_signal.emit("대기열 소모 → 대기 시작(잔여)")
                            self._schedule_wait_from_edge(t0)
                        else:
                            self._schedule_phase("arm_edge", 0.0)

            # 대기/모션 중 들어온 남은 edges는 타임스탬프 큐에 적재
            if phase_before in ("wait_before", "wait_motion_done") and edges:
                max_q = int(self._preset.get("max_queue", 0) or 0)
                for t in edges:
                    if max_q > 0 and len(self._edge_queue) >= max_q:
                        break
                    self._edge_queue.append(t)
                    self._last_accepted_edge_time = t
                self.status_signal.emit(f"엣지 적재(대기열={len(self._edge_queue)})")

            self._prev_sensor = sensor_val
            remain = max(0.0, self._phase_until - now)
            self.status_signal.emit(
                f"[센서:{sensor_val}] 단계:{self._next_phase} 남은:{remain:0.1f}s, 대기열:{len(self._edge_queue)}"
            )
            return

        # -------- schedule mode (기존 유지: 필요시 동일하게 wait_motion_done으로 바꿀 수 있음) --------
        if now >= self._phase_until:
            if self._next_phase == "wait_before":
                mf = self._preset["motion_file"]
                cs = int(self._preset.get("chunk_size", 0) or 0)
                cd = int(self._preset.get("chunk_delay_ms", 0) or 0)
                ok = self.motion_executor_fn(mf, cs, cd)
                if ok:
                    self.status_signal.emit(f"모션 실행: {os.path.basename(mf)}")
                    self._schedule_phase("wait_motion_done", 0.0)
                else:
                    self.status_signal.emit("모션 실행 실패: 경로/파라미터 확인")
                    self._schedule_phase("wait_before", 2.0)
            elif self._next_phase == "wait_motion_done":
                if not self._is_busy_fn():
                    self._schedule_phase("wait_before", self._preset.get("wait_before_s", 0.0))

        remain = max(0.0, self._phase_until - now)
        self.status_signal.emit(f"[센서:{sensor_val}] 단계:{self._next_phase} 남은:{remain:0.1f}s")


def _guess_workspace_root() -> str:
    """
    workspace 루트를 추론.
    우선순위:
      1) 환경변수 SMART_WS_DIR
      2) __file__ 경로에 /src/ 가 있으면 그 앞을 <ws>로 사용
      3) __file__ 경로에 /install/ 이 있으면 그 앞을 <ws>로 사용
      4) fallback: cwd
    """
    env = os.environ.get("SMART_WS_DIR")
    if env:
        return str(Path(env).expanduser().resolve())

    try:
        here = Path(__file__).resolve()
        parts = here.parts

        # 2) .../<ws>/src/... 형태
        if "src" in parts:
            i = parts.index("src")
            if i > 0:
                return str(Path(*parts[:i]).resolve())

        # 3) .../<ws>/install/... 형태 (현재 문제 케이스)
        if "install" in parts:
            i = parts.index("install")
            if i > 0:
                return str(Path(*parts[:i]).resolve())

    except Exception:
        pass

    return str(Path.cwd().resolve())


def _default_motions_dir() -> str:
    ws = Path(_guess_workspace_root())
    return str((ws / "motions").resolve())


# ---------------- ROS Runner Node ----------------
class RunnerNode(Node):
    def __init__(self):
        super().__init__('motion_gui_runner')

        # Params
        self.declare_parameter('reflect_topic', '/reflect_sensor')
        self.declare_parameter('trigger_mode', 'falling_delay')  # legacy (차단 예정)
        self.declare_parameter('post_clear_delay_ms', 0)
        self.declare_parameter('cooldown_ms', 2000)
        self.declare_parameter('debounce_ms', 150)
        self.declare_parameter('initial_motion', '')

        self.declare_parameter('reflect_reliability', 'BEST_EFFORT')  # or 'RELIABLE'
        self.declare_parameter('reflect_depth', 5)

        self.declare_parameter('run_mode', 'inline')  # 'inline' | 'process'
        self.run_mode = self.get_parameter('run_mode').get_parameter_value().string_value
        self.servo_ui = {'t1': 0.08, 't2': 0.05, 'gain': 0.05, 'alpha': 0.003}

        self.last_motion_duration_s = None
        self.duration_sub = self.create_subscription(
            Float32, "/motion_executor/last_duration_s", self._on_motion_duration, 10
        )

        # 실센서 상승엣지 이벤트(타임스탬프) 큐 + 락
        self._edge_events = deque()
        self._edge_events_max = 100
        self._edge_lock = threading.Lock()

        # 자동모드 플래그/허용 토큰
        self._edge_capture_enabled = False
        self._auto_mode_active = False
        self._allow_controller_run_once = False

        # ✅ 하드코딩 제거: motions 디렉터리 자동 추론
        motions_dir = _default_motions_dir()
        self.declare_parameter('home_yaml',   str(Path(motions_dir) / "home_motion.yaml"))
        self.declare_parameter('standby_yaml', str(Path(motions_dir) / "standby_motion.yaml"))
        self.declare_parameter('default_dir', motions_dir)
        self._resolved_home_yaml = self._resolve_home_yaml_path()

        # Unity 저장 디렉터리도 workspace 기준으로 (원하면 ~/Downloads 등으로 변경 가능)
        self.declare_parameter('unity_save_dir', str(Path(_guess_workspace_root()) / "unity_motions"))

        self.declare_parameter(
            'launch_cmd_template',
            'ros2 run rb_test test_cobot_publisher --ros-args -p motion_file:="{path}" -p auto_load:=false'
        )

        # Home/Standby defaults (한 번만 선언)
        self.declare_parameter('home_pose_arr', [0.0, -45.00, 137.00, 0.00, -90.00, 0.0])
        self.declare_parameter('home_move', 'lin')
        self.declare_parameter('home_coord', 'base')
        self.declare_parameter('home_speed', 20.0)
        self.declare_parameter('home_accel', 20.0)

        self.declare_parameter('standby_pose_arr', [0.0, -45.00, 137.00, 0.00, -90.00, 0.0])
        self.declare_parameter('standby_move',  'lin')
        self.declare_parameter('standby_coord', 'base')
        self.declare_parameter('standby_speed', 20.0)
        self.declare_parameter('standby_accel', 20.0)

        # ---- Unity Live(실시간) 파라미터 (한 번만 선언) ----
        self.declare_parameter('unity_listen', '0.0.0.0')
        self.declare_parameter('unity_port', 10001)
        self.declare_parameter('unity_allow', '')
        self.declare_parameter('unity_robot_ip', '192.168.1.13')
        self.declare_parameter('unity_axis_map', ['y','x','x','x','y','x'])
        self.declare_parameter('unity_servo_Ts', 0.008)
        self.declare_parameter('unity_filter_alpha', 1.0)

        self.declare_parameter('unity_playback_dt_default', 0.05)
        self.declare_parameter('unity_playback_use_file_dt', False)
        self.declare_parameter('unity_playback_use_file_servo', False)
        self.declare_parameter('unity_playback_servo_t1', 0.05)
        self.declare_parameter('unity_playback_servo_t2', 0.05)
        self.declare_parameter('unity_playback_servo_gain', 0.1)
        self.declare_parameter('unity_playback_servo_alpha', 0.03)
        self.declare_parameter('unity_playback_filter_alpha', 1.0)

        self.declare_parameter('robot_model', 'rb10-1300e')

        self.declare_parameter('unity_tail_hold_s', 0.3)
        self.declare_parameter('unity_stop_after_playback', False)

        self.declare_parameter('tcp_listen_host', '0.0.0.0')
        self.declare_parameter('tcp_listen_port', 12001)
        self.declare_parameter('ps_value_topic', '/ps_value')
        self.declare_parameter('ps_quiet_s', 0.5)

        u_listen = self.get_parameter('unity_listen').get_parameter_value().string_value
        u_port   = int(self.get_parameter('unity_port').value)
        u_allow  = self.get_parameter('unity_allow').get_parameter_value().string_value or None
        u_robot  = self.get_parameter('unity_robot_ip').get_parameter_value().string_value
        u_save   = self.get_parameter('unity_save_dir').get_parameter_value().string_value
        u_axis   = list(self.get_parameter('unity_axis_map').value)
        u_Ts     = float(self.get_parameter('unity_servo_Ts').value)
        u_filt   = float(self.get_parameter('unity_filter_alpha').value)


        self._unity_wpdt_default = float(self.get_parameter('unity_playback_dt_default').value)
        self._unity_use_file_dt = bool(self.get_parameter('unity_playback_use_file_dt').value)
        self._unity_use_file_servo = bool(self.get_parameter('unity_playback_use_file_servo').value)
        self._unity_play_t1 = float(self.get_parameter('unity_playback_servo_t1').value)
        self._unity_play_t2 = float(self.get_parameter('unity_playback_servo_t2').value)
        self._unity_play_gain = float(self.get_parameter('unity_playback_servo_gain').value)
        self._unity_play_alpha = float(self.get_parameter('unity_playback_servo_alpha').value)
        self._unity_play_filt = float(self.get_parameter('unity_playback_filter_alpha').value)
        self._robot_model = str(self.get_parameter('robot_model').get_parameter_value().string_value).lower()

        self._unity_tail_hold_s = float(self.get_parameter('unity_tail_hold_s').value)
        self._unity_stop_after_playback = bool(self.get_parameter('unity_stop_after_playback').value)

        # 모델 프리셋 적용(런타임 강제)
        self._apply_robot_preset()

        # GUI
        self.app = QApplication([])
        self.win = Window(self)
        try:
            self.win.set_selected_path(self.selected_path)
        except Exception:
            pass

        # ▶ TCP 서버 시작 (값 수신: 줄/공백/바이너리에서 정수 파싱)
        try:
            host = str(self.get_parameter('tcp_listen_host').get_parameter_value().string_value)
            port = int(self.get_parameter('tcp_listen_port').value)
            self._start_tcp_server(host, port)
            self.get_logger().info(f"[TCP] listening {host}:{port} for control values (-2..2,10)")
        except Exception as e:
            self.get_logger().error(f"[TCP] server start failed: {e}")

        # ▶ P_S 문자열 토픽 수신 (예: "P_S 2" 또는 "P_S none")
        try:
            ps_topic = self.get_parameter('ps_value_topic').get_parameter_value().string_value or '/ps_value'
            self._ps_quiet_s = float(self.get_parameter('ps_quiet_s').value)
            # 최근 수신값/시각 추적 및 제품 각도 값 대기 후 큐 적재 플래그
            self._ps_latest_value = None
            self._ps_last_time = 0.0
            self._ps_enqueued_since_last = True
            # 중복 적재 방지용 락 및 마지막 적재값 추적
            self._ps_quiet_lock = threading.Lock()
            self._ps_last_enqueued_value = None

            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self._ps_sub = self.create_subscription(String, ps_topic, self._on_ps_value, qos)
            # 제품 각도 값 대기 감지 타이머(50ms)
            self._ps_timer = self.create_timer(0.05, self._ps_quiet_check)
            self.get_logger().info(f"[P_S] subscribing to {ps_topic} (String), quiet={self._ps_quiet_s}s")
        except Exception as e:
            self.get_logger().error(f"[P_S] subscribe failed: {e}")

        # Unity 브리지 생성 및 서버 시작(비동기)
        self.unity_save_dir = u_save
        try:
            os.makedirs(self.unity_save_dir, exist_ok=True)
        except Exception:
            pass
        self.unity_bridge = UnityTcpBridge(
            robot_ip=u_robot,
            listen_host=u_listen,
            listen_port=u_port,
            allowed_client_ip=u_allow,
            joint_axis_map=u_axis,
            joint_round_decimals=2,
            servo_Ts=u_Ts,
            filter_alpha=u_filt,
        )
        def _run_unity_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.unity_bridge.start())
            finally:
                try:
                    loop.stop()
                except Exception:
                    pass
                try:
                    loop.close()
                except Exception:
                    pass
        self._unity_thread = threading.Thread(target=_run_unity_server, daemon=True)
        self._unity_thread.start()
        self.get_logger().info(
            f"[bridge] up listen={u_listen}:{u_port} robot_ip={u_robot} mode=joint Ts={u_Ts} joint_axis_map={u_axis}"
        )


        # Unity 브리지 생성 직후 초기 파라미터 동기화
        try:
            self.unity_bridge.set_servo_overrides(
                t1=self._unity_play_t1, t2=self._unity_play_t2,
                gain=self._unity_play_gain, alpha=self._unity_play_alpha
            )
            if hasattr(self.unity_bridge, "set_filter_alpha"):
                self.unity_bridge.set_filter_alpha(self._unity_play_filt)
            if hasattr(self.unity_bridge, "set_tail_hold_s"):
                self.unity_bridge.set_tail_hold_s(self._unity_tail_hold_s)
            if hasattr(self.unity_bridge, "set_stop_after_playback"):
                self.unity_bridge.set_stop_after_playback(self._unity_stop_after_playback)
            self.unity_bridge.set_forwarding(True)
        except Exception as e:
            self.get_logger().warning(f"Unity bridge init sync failed: {e}")

    def _resolve_home_yaml_path(self) -> str:
        """
        home_yaml 실행 경로를 확정한다.
        - 파라미터 home_yaml이 유효하면 사용
        - 아니면 <ws>/motions/home_motion.yaml
        - 아니면 <ws> 아래에서 home_motion.yaml 검색
        """
        ws = Path(_guess_workspace_root()).resolve()

        # 1) 파라미터 값 우선
        try:
            home_yaml = (self.get_parameter('home_yaml').value or "").strip()
        except Exception:
            home_yaml = ""

        if home_yaml:
            p = Path(home_yaml).expanduser()
            if p.is_file():
                return str(p.resolve())

        # 2) 기본 위치: <ws>/motions/home_motion.yaml
        p2 = (ws / "motions" / "home_motion.yaml")
        if p2.is_file():
            return str(p2.resolve())

        # 3) workspace 전체 검색(깊이는 적당히 제한)
        found = _find_file_under(ws, "home_motion.yaml", max_depth=6)
        if found:
            self.get_logger().warn(f"[HOME] home_motion.yaml 자동 탐색: {found}")
            return str(found)

        # 4) 못 찾으면 빈 문자열 → run_home에서 /tmp 임시 yaml 생성 fallback
        self.get_logger().warn("[HOME] home_motion.yaml을 찾지 못했습니다. (정책: 홈은 파일 기반 only)")
        return ""

    def _start_tcp_server(self, host: str, port: int):
        import socket
        def _srv():
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(5)
            while True:
                try:
                    conn, addr = srv.accept()
                    with conn:
                        buf = b""
                        while True:
                            chunk = conn.recv(1024)
                            if not chunk:
                                break
                            buf += chunk
                            # 줄바꿈 기준으로 분할
                            while b"\n" in buf or b"\r" in buf:
                                for sep in (b"\n", b"\r"):
                                    parts = buf.split(sep, 1)
                                    if len(parts) == 2:
                                        line = parts[0]
                                        buf = parts[1]
                                        break
                                try:
                                    s = line.decode('utf-8', errors='ignore').strip()
                                    if not s:
                                        continue
                                    ls = s.lower()
                                    handled = False
                                    # 형식: data: '{"signal":"-1", ...}' 또는 유사 JSON 포함 라인 처리
                                    if 'data:' in ls and '{' in s and '}' in s:
                                        try:
                                            jstr = s[s.index('{'): s.rindex('}')+1]
                                            obj = json.loads(jstr)
                                            sig = obj.get('signal')
                                            if sig is not None and hasattr(self.win, 'auto_ctrl'):
                                                if isinstance(sig, str) and sig.strip().lower() == 'none':
                                                    self.win.auto_ctrl.set_pending_value(None)
                                                    self.gui_set_status("TCP 수신 signal: none (coalesced)")
                                                else:
                                                    try:
                                                        v = int(sig)
                                                        if v in (-2, -1, 0, 1, 2):
                                                            self.win.auto_ctrl.set_pending_value(v)
                                                            self.gui_set_status(f"TCP 수신 signal: {v} (coalesced)")
                                                    except Exception:
                                                        pass
                                            handled = True
                                        except Exception:
                                            pass
                                    if handled:
                                        continue
                                    # [p_s] 접두/토큰 기반: 마지막 값만 보류
                                    if ls.startswith('[p_s]'):
                                        s2 = s[len('[p_s]'):].strip()
                                        toks = s2.replace(',', ' ').split()
                                    else:
                                        toks = s.replace(',', ' ').split()
                                    for tok in toks:
                                        t = tok.strip().lower()
                                        if not t:
                                            continue
                                        if t == 'p_s':
                                            continue
                                        if t == 'none':
                                            if hasattr(self.win, 'auto_ctrl'):
                                                self.win.auto_ctrl.set_pending_value(None)
                                            self.gui_set_status("TCP 수신 값: none (coalesced)")
                                        else:
                                            try:
                                                v = int(t)
                                                if hasattr(self.win, 'auto_ctrl') and v in (-2, -1, 0, 1, 2):
                                                    self.win.auto_ctrl.set_pending_value(v)
                                                    self.gui_set_status(f"TCP 수신 값: {v} (coalesced)")
                                            except Exception:
                                                pass
                                except Exception:
                                    pass
                except Exception:
                    time.sleep(0.1)
        t = threading.Thread(target=_srv, daemon=True)
        t.start()

    def _on_ps_value(self, msg: String):
        """String 토픽으로 들어온 P_S 명령을 파싱하여 값 큐에 주입.
        허용 포맷:
          - "P_S 2" / "P_S -1" / "P_S none"
          - "[p_s]2" / "[p_s]none" (소문자, 공백 없음) ← 신규 지원
          - 접두사 없이 "2" 등만 오는 경우도 수용
          - 공백/콤마 구분자 허용 (여러 값 연속 처리)
          - JSON 문자열: '{"signal": "-1", ...}' → signal만 사용 (burst coalesce)
        """
        try:
            raw = str(getattr(msg, 'data', '') or '')
            s = raw.strip().lower()
            if not s:
                return
            # JSON payload 우선 처리: '{...}' 형태이고 signal 키가 있으면 최근값만 갱신
            if s.startswith('{') and 'signal' in s:
                try:
                    obj = json.loads(raw)
                    sig = obj.get('signal')
                    fed = []
                    if sig is not None:
                        if isinstance(sig, str) and sig.strip().lower() == 'none':
                            with self._ps_quiet_lock:
                                self._ps_latest_value = None
                                self._ps_last_time = time.time()
                                self._ps_enqueued_since_last = False
                            fed.append('none')
                        else:
                            try:
                                v = int(sig)
                                if v in (-2, -1, 0, 1, 2):
                                    with self._ps_quiet_lock:
                                        self._ps_latest_value = v
                                        self._ps_last_time = time.time()
                                        self._ps_enqueued_since_last = False
                                    fed.append(str(v))
                            except Exception:
                                pass
                    if fed:
                        self.gui_set_status(f"P_S(JSON) 최신값: {' '.join(fed)}")
                    return
                except Exception:
                    # JSON 파싱 실패 시 토큰 방식으로 폴백
                    pass
            # [p_s] 접두사(공백 없이 붙은 형태) 우선 제거
            if s.startswith('[p_s]'):
                s = s[len('[p_s]'):].strip()
            toks = s.replace(',', ' ').split()
            if not toks:
                return
            # 접두사 처리
            if toks[0].lower() == 'p_s':
                toks = toks[1:]
            fed = []
            for t in toks:
                tt = t.strip().lower()
                if tt == 'none':
                    with self._ps_quiet_lock:
                        self._ps_latest_value = None
                        self._ps_last_time = time.time()
                        self._ps_enqueued_since_last = False
                    fed.append('none')
                else:
                    try:
                        v = int(tt)
                        if v in (-2, -1, 0, 1, 2):
                            with self._ps_quiet_lock:
                                self._ps_latest_value = v
                                self._ps_last_time = time.time()
                                self._ps_enqueued_since_last = False
                            fed.append(str(v))
                        else:
                            # 범위 외 값은 무시
                            pass
                    except Exception:
                        pass
            if fed:
                self.gui_set_status(f"P_S 최신값: {' '.join(fed)}")
        except Exception:
            pass

    def _ps_quiet_check(self):
        """최근 P_S 수신 후 제품 각도 값 대기가 지나면 마지막 값을 큐에 적재."""
        try:
            # 중복 적재 방지를 위해 락을 사용하고 플래그를 선설정
            with self._ps_quiet_lock:
                if self._ps_last_time <= 0.0:
                    return
                if self._ps_enqueued_since_last:
                    return
                if (time.time() - float(self._ps_last_time)) < float(self._ps_quiet_s):
                    return
                v = self._ps_latest_value
                # 선플래그 설정으로 타이머 레이스 방지
                self._ps_enqueued_since_last = True
                self._ps_last_enqueued_value = v
            # 락 밖에서 실제 보류값 설정 수행(큐 사용 안 함)
            if hasattr(self.win, 'auto_ctrl'):
                self.win.auto_ctrl.set_pending_value(v)
            disp = 'none' if v is None else str(v)
            self.gui_set_status(f"P_S 제품 각도 값 대기({self._ps_quiet_s:.2f}s) → 최근값 설정: {disp}")
        except Exception:
            pass

    def _apply_robot_preset(self):
        """로봇 모델에 맞춘 ServoJ/필터/재생 dt 프리셋을 적용."""
        m = (self._robot_model or "").lower()
        # 기본: RB10-1300E (현 설정 유지)
        if "rb5" in m:
            # RB5-850E 권장 시작값(더 빠르고 잔류 억제 강화)
            self._unity_play_t1 = 0.040     # 더 짧게 → 응답 향상
            self._unity_play_t2 = 0.040
            self._unity_play_gain = 0.055   # 약간 상향
            self._unity_play_alpha = 0.0045 # 소폭 상향
            self._unity_play_filt = 0.9     # 약한 저역통과로 잔떨림 억제
            # 재생 간격을 0.04s로 소폭 단축(속도 향상, 과도 시 0.05로 복귀)
            if not self._unity_use_file_dt:
                self._unity_wpdt_default = 0.04
            # 꼬리 홀드/종료 강화
            self._unity_tail_hold_s = 0.6
            self._unity_stop_after_playback = True
            self.get_logger().info("[preset] RB5-850E ServoJ preset applied")
        # else: RB10 기본 유지

        # State
        self.last_state = None         # 0/1

        # 가상 센서(펄스)
        self._sensor_force_value = None
        self._sensor_force_until = 0.0

        # read params
        self.topic = self.get_parameter('reflect_topic').value
        self.trigger_mode = self.get_parameter('trigger_mode').value
        self.post_clear_delay_ms = int(self.get_parameter('post_clear_delay_ms').value)
        self.cooldown_ms = int(self.get_parameter('cooldown_ms').value)
        self.debounce_ms = int(self.get_parameter('debounce_ms').value)
        self.launch_cmd_template = self.get_parameter('launch_cmd_template').value

        self.selected_path = self.get_parameter('initial_motion').value
        self.default_dir = self.get_parameter('default_dir').value

        self.home_pose_arr = list(self.get_parameter('home_pose_arr').value)
        self.home_move  = self.get_parameter('home_move').get_parameter_value().string_value
        self.home_coord = self.get_parameter('home_coord').get_parameter_value().string_value
        self.home_speed = float(self.get_parameter('home_speed').value)
        self.home_accel = float(self.get_parameter('home_accel').value)
        self.home_yaml = self.get_parameter('home_yaml').get_parameter_value().string_value
        self.get_logger().info(f"[INIT] home_yaml loaded: {self.home_yaml}, exists: {os.path.exists(self.home_yaml) if self.home_yaml else False}")

        self.standby_yaml  = self.get_parameter('standby_yaml').get_parameter_value().string_value
        self.get_logger().info(f"[INIT] standby_yaml loaded: {self.standby_yaml}, exists: {os.path.exists(self.standby_yaml) if self.standby_yaml else False}")
        self.standby_pose_arr = list(self.get_parameter('standby_pose_arr').value)
        self.standby_move  = self.get_parameter('standby_move').get_parameter_value().string_value
        self.standby_coord = self.get_parameter('standby_coord').get_parameter_value().string_value
        self.standby_speed = float(self.get_parameter('standby_speed').value)
        self.standby_accel = float(self.get_parameter('standby_accel').value)

        # State
        self.last_state = None         # 0/1
        self.last_event_ms = 0
        self.seen_one = False
        self.delay_timer = None

        self.last_run_ms = 0
        self.proc = None
        self._busy_inline = False

        # GUI 갱신 버퍼
        self._sensor_view = None
        self._pending_status = None
        # 상승엣지 간 간격 표시용
        self._last_rising_time = None     # 마지막 상승엣지의 time.time() (초)
        self._pending_interval_s = None   # UI 스레드에서 표시할 대기 값

        # ROS (QoS)
        rel_param = self.get_parameter('reflect_reliability').get_parameter_value().string_value.upper()
        depth_param = int(self.get_parameter('reflect_depth').value)
        reliability = (QoSReliabilityPolicy.RELIABLE
                       if rel_param == 'RELIABLE' else QoSReliabilityPolicy.BEST_EFFORT)
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth_param,
            reliability=reliability,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Bool, self.topic, self._on_sensor_msg, sensor_qos)

        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "/motion_gui/status", status_qos)
        self.proc_check_timer = self.create_timer(0.3, self._check_proc)

        self.get_logger().info(
            f"GUI Runner listening {self.topic}, trigger={self.trigger_mode}, "
            f"debounce={self.debounce_ms}ms, cooldown={self.cooldown_ms}ms"
        )

        # Executor
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

        self.exec_node = None
        if self.run_mode == 'inline':
            self.exec_node = MotionExecutor()
            self._exec.add_node(self.exec_node)

        # (중복 제거) GUI 생성은 상단에서 1회만 수행

        # ▶ 시작 지연(스핀박스) 초기값을 강제로 0.0으로 맞추고 내부값도 동기화
        try:
            self.win.spin_delay.blockSignals(True)
            self.win.spin_delay.setValue(0.0)
            self.win.spin_delay.blockSignals(False)
            self.set_start_delay_seconds(0.0)  # self.post_clear_delay_ms = 0 반영
        except Exception:
            pass

    # ---------- Emergency Stop ----------
    def emergency_stop(self):
        try:
            # 0) Unity 브리지 재생/포워딩 즉시 중단
            try:
                if getattr(self, "unity_bridge", None):
                    # 재생 중이면 중단
                    if hasattr(self.unity_bridge, "is_playing") and self.unity_bridge.is_playing():
                        if hasattr(self.unity_bridge, "stop_playback"):
                            self.unity_bridge.stop_playback()
                    # 라이브 포워딩 일시 중단(안전)
                    try:
                        self.unity_bridge.set_forwarding(False)
                    except Exception:
                        pass
            except Exception:
                pass

            # 1) Inline 모드: 실행 중이면 즉시 취소 신호
            if self.exec_node is not None:
                try:
                    self.exec_node.request_cancel()
                except Exception:
                    pass

            # 2) 로봇에 즉시 정지 명령
            try:
                cobot.MotionHalt()
            except Exception:
                pass

            # 3) 컨트롤러 버퍼 비우기 (가능한 경우)
            try:
                if hasattr(cobot, 'MoveJB_Clear'):
                    cobot.MoveJB_Clear()
            except Exception:
                pass
            try:
                if hasattr(cobot, 'MovePB_Clear'):
                    cobot.MovePB_Clear()
            except Exception:
                pass

            # 4) 프로세스 모드: 외부 프로세스 종료
            if self.proc is not None:
                try:
                    if self.proc.poll() is None:
                        self.proc.terminate()
                        try:
                            self.proc.wait(timeout=1.0)
                        except Exception:
                            self.proc.kill()
                except Exception:
                    pass
                finally:
                    self.proc = None

            # 5) 자동모드 중이면 컨트롤러까지 정지
            try:
                # 엣지 캡처 OFF + 큐 비우기
                self.set_edge_capture_enabled(False)
                with getattr(self, "_edge_lock", threading.Lock()):
                    if hasattr(self, "_edge_events"):
                        self._edge_events.clear()
                # 오토 컨트롤러 동작 중이면 중단
                if getattr(self, "win", None) and hasattr(self.win, "auto_ctrl"):
                    self.win.auto_ctrl.stop()
            except Exception:
                pass

            # 6) 보류값/큐 초기화(중복 송신 방지)
            try:
                self.clear_value_queue(also_reset_pending=True)
            except Exception:
                pass

            # 7) 내부 플래그 정리
            self._busy_inline = False
            self._allow_controller_run_once = False

            # 8) 상태 로그
            self.gui_set_status("비상정지: 모든 동작/재생/자동모드 중단")
        except Exception as e:
            self.gui_set_status(f"비상정지 실패: {e}")

    def _on_motion_duration(self, msg: Float32):
        try:
            self.last_motion_duration_s = float(msg.data)
            # 상태 라벨에 수행시간 표시
            self.gui_set_status(f"모션 완료 (수행시간: {self.last_motion_duration_s:.2f} s)")
        except Exception:
            pass

    def set_start_delay_seconds(self, sec: int):
        try:
            sec = max(0.0, float(sec))
        except Exception:
            sec = 0.0
        self.post_clear_delay_ms = int(round(sec * 1000.0))

    def run_motion_file(self, motion_file: str, chunk_size: int = 0, chunk_delay_ms: int = 0) -> bool:
        # 컨트롤러 기원 실행 1회 허용
        self._allow_controller_run_once = True
        try:
            if not motion_file or not os.path.exists(motion_file):
                self.gui_set_status("모션 파일 경로가 없습니다.")
                return False
            self.set_selected_path(motion_file)
            # 자동모드: 브리지 재생 사용
            return self._maybe_run_motion_via_bridge(motion_file)
        except Exception as e:
            self.gui_set_status(f"실행 실패: {e}")
            return False
        
    def is_motion_busy(self) -> bool:
        """브리지 재생이 진행 중인지 확인(수동/자동 공용)."""
        try:
            if getattr(self, "_busy_inline", False):
                return True
            if getattr(self, "unity_bridge", None) and hasattr(self.unity_bridge, "is_playing"):
                return bool(self.unity_bridge.is_playing())
        except Exception:
            pass
        return False
    
    def _maybe_run_motion_via_bridge(self, path: str = None) -> bool:
        """UnityTcpBridge의 playback_start로 재생(step-hold). 수동/자동모드에서 사용."""
        effective_path = path or self.selected_path
        if not effective_path or not os.path.exists(effective_path):
            self.gui_set_status("모션 파일 미선택/경로 없음")
            return False
        now = self.now_ms()
        if (now - self.last_run_ms) < self.cooldown_ms:
            self.gui_set_status("쿨다운 중 → 무시")
            return False
        if self._auto_mode_active and not self._allow_controller_run_once:
            self.gui_set_status("실행 차단: 자동모드 활성 중 (컨트롤러 외 실행 금지)")
            return False
        self._allow_controller_run_once = False
        # 파싱 → joints 시퀀스 + wp_dt
        try:
            waypoints, params = self._extract_waypoints_and_params(effective_path)
        except Exception as e:
            self.gui_set_status(f"파싱 실패: {e}")
            return False
        seq = []
        for wp in waypoints:
            q = wp.get("joints") or wp.get("q")
            if isinstance(q, (list, tuple)) and len(q) >= 6:
                seq.append([float(q[i]) for i in range(6)])
        if not seq:
            self.gui_set_status("joint 웨이포인트가 없습니다.")
            return False
        # 1) 재생 간격: 기본은 rec_gui처럼 고정 dt 사용, 원할 때만 파일 wp_dt 사용
        if getattr(self, "_unity_use_file_dt", False):
            wp_dt = float((params or {}).get("wp_dt", self._unity_wpdt_default))
        else:
            wp_dt = float(self._unity_wpdt_default)

        # 송신 간격을 wp_dt로 고정(재생 동안)
        from rb_test import cobot
        cobot.set_send_min_interval(wp_dt)

        # 2) ServoJ 파라미터: rec_gui 방식으로 강제(기본). 필요 시 파일값 사용 옵션 제공.
        p = params or {}
        use_file_servo = bool(self._unity_use_file_servo)
        try:
            # 현재 유효값 결정: 파일 사용 옵션이면 파일값, 아니면 _unity_play_* (GUI가 갱신)
            eff_t1 = self._unity_play_t1
            eff_t2 = self._unity_play_t2
            eff_g  = self._unity_play_gain
            eff_a  = self._unity_play_alpha
            if use_file_servo:
                t1 = p.get("servo_t1"); t2 = p.get("servo_t2")
                gn = p.get("servo_gain"); al = p.get("servo_alpha")
                eff_t1 = float(t1) if t1 is not None else eff_t1
                eff_t2 = float(t2) if t2 is not None else eff_t2
                eff_g  = float(gn) if gn is not None else eff_g
                eff_a  = float(al) if al is not None else eff_a
            # 재생 전 브리지에 적용
            self.unity_bridge.set_servo_overrides(t1=eff_t1, t2=eff_t2, gain=eff_g, alpha=eff_a)
            if use_file_servo:
                t1 = p.get("servo_t1"); t2 = p.get("servo_t2")
                gn = p.get("servo_gain"); al = p.get("servo_alpha")
                self.unity_bridge.set_servo_overrides(
                    t1=float(t1) if t1 is not None else self._unity_play_t1,
                    t2=float(t2) if t2 is not None else self._unity_play_t2,
                    gain=float(gn) if gn is not None else self._unity_play_gain,
                    alpha=float(al) if al is not None else self._unity_play_alpha,
                )
            else:
                # 파일 servo 무시 → rec_gui 기본 강제
                self.unity_bridge.set_servo_overrides(
                    t1=self._unity_play_t1, t2=self._unity_play_t2,
                    gain=self._unity_play_gain, alpha=self._unity_play_alpha
                )
            # 필터도 rec_gui와 동일하게 강제
            if hasattr(self.unity_bridge, "set_filter_alpha"):
                self.unity_bridge.set_filter_alpha(self._unity_play_filt)
        except Exception as e:
            self.get_logger().warning(f"apply playback servo params failed: {e}")

        try:
            self.unity_bridge.set_forwarding(True)
        except Exception:
            pass
        self.last_run_ms = now
        self._busy_inline = True
        def _run_bridge():
            try:
                self.get_logger().info(f"[bridge] playback via GUI: N={len(seq)}, dt={wp_dt:.3f}, "
                                       f"use_file_servo={use_file_servo}, "
                                       f"t1={eff_t1}, t2={eff_t2}, gain={eff_g}, alpha={eff_a}, "
                                       f"filt={self._unity_play_filt}")
                # 꼬리 홀드/종료 옵션을 매 실행 전에 재확인/적용
                if hasattr(self.unity_bridge, "set_tail_hold_s"):
                    self.unity_bridge.set_tail_hold_s(self._unity_tail_hold_s)
                if hasattr(self.unity_bridge, "set_stop_after_playback"):
                    self.unity_bridge.set_stop_after_playback(self._unity_stop_after_playback)
                # 재생 시작
                self.unity_bridge.playback_start(seq, wp_dt=wp_dt)
                t0 = time.monotonic()
                # 재생 종료까지 대기: is_playing() 지원 시 polling, 아니면 안전 타임아웃으로 대기
                max_wait = wp_dt * max(0, len(seq) - 1) + 3.0
                if hasattr(self.unity_bridge, "is_playing"):
                    while True:
                        if not self.unity_bridge.is_playing():
                            break
                        if (time.monotonic() - t0) >= max_wait:
                            break
                        time.sleep(0.01)
                else:
                    time.sleep(max(0.0, max_wait))
                elapsed = time.monotonic() - t0
                self.gui_set_status(f"모션 완료 (수행시간: {elapsed:.2f} s)")
            except Exception as e:
                self.gui_set_status(f"브리지 재생 실패: {e}")
            finally:
                self._busy_inline = False
        threading.Thread(target=_run_bridge, daemon=True).start()
        return True

    def set_edge_capture_enabled(self, enabled: bool):
        self._edge_capture_enabled = bool(enabled)
        self._auto_mode_active = bool(enabled)
        if not enabled:
            # 자동모드 OFF 시, 과거 엣지 폐기 (ON하자마자 즉시 실행 방지)
            with self._edge_lock:
                self._edge_events.clear()

    # 외부(노드)에서 TCP로 받은 값을 주입
    def feed_value(self, v: int):
        try:
            self._value_queue.append(int(v))
        except Exception:
            pass

    def clear_value_queue(self, also_reset_pending: bool = True):
        """수동 큐 초기화: 값 큐 비우기 + P_S 제품 각도 값 대기/보류 상태 초기화.
        also_reset_pending=True이면 보류값/플래그도 함께 초기화.
        """
        try:
            # 값 큐 비우기
            if hasattr(self, '_value_queue'):
                try:
                    self._value_queue.clear()
                except Exception:
                    # deque가 아닐 경우 호환 처리
                    self._value_queue = type(self._value_queue)()
            # P_S 상태 초기화
            with getattr(self, '_ps_quiet_lock', threading.Lock()):
                self._ps_latest_value = None
                self._ps_last_time = 0.0
                self._ps_enqueued_since_last = True
                self._ps_last_enqueued_value = None
            # 자동모드 컨트롤러 보류값도 초기화(있을 때만)
            if also_reset_pending and hasattr(self.win, 'auto_ctrl'):
                try:
                    if hasattr(self.win.auto_ctrl, 'clear_pending'):
                        self.win.auto_ctrl.clear_pending()
                    elif hasattr(self.win.auto_ctrl, 'set_pending_value'):
                        self.win.auto_ctrl.set_pending_value(None)
                except Exception:
                    pass
            self.gui_set_status("값 큐 초기화 완료(보류/제품 각도 값 대기 상태 리셋)")
        except Exception as e:
            self.gui_set_status(f"값 큐 초기화 실패: {e}")

    # ---------- Utilities ----------
    @staticmethod
    def _resolve_motion_path(path: str) -> str:
        path = os.path.expanduser(path)
        if os.path.isfile(path):
            return path
        root, _ = os.path.splitext(path)
        for alt in (root + ".json", root + ".yaml", root + ".yml"):
            if os.path.isfile(alt):
                return alt
        raise FileNotFoundError(f"Motion file not found: {path}")

    def now_ms(self) -> int:
        return int(time.monotonic() * 1000)

    # ---------- GUI helpers ----------
    def gui_set_status(self, s: str):
        try:
            self.status_pub.publish(String(data=s))
        except Exception:
            pass
        self._pending_status = s

    def gui_set_interval(self, seconds: float):
        """상승엣지 간 간격(초)을 UI에 반영."""
        try:
            self.win.set_interval(float(seconds))
        except Exception:
            pass
        self._pending_interval_s = float(seconds)

    def gui_set_state(self, v: int):
        self._sensor_view = v

    def set_selected_path(self, path: str):
        self.selected_path = path
        self.win.set_selected_path(path)

    # ---------- Sensor handling ----------
    def _on_sensor_msg(self, msg: Bool):
        v = 1 if msg.data else 0
        now = self.now_ms()

        # edge-aware debounce
        if (self.last_state is not None
            and v == self.last_state
            and (now - self.last_event_ms) < self.debounce_ms):
            return

        rising = (self.last_state == 0 and v == 1) if self.last_state is not None else (v == 1)
        self.last_state = v
        self.last_event_ms = now
        self.gui_set_state(v)  # UI 표시는 항상 갱신


        # 상승엣지 간 간격 계산 → UI 표시 (자동모드 여부와 무관하게 갱신)
        if rising:
            now_sec = time.time()
            if self._last_rising_time is not None:
                dt = now_sec - self._last_rising_time
                if dt >= 0:
                    self.gui_set_interval(dt)
            self._last_rising_time = now_sec

        # ★ 자동모드 OFF면 여기서 종료 (레거시 트리거 완전 차단)
        if not self._auto_mode_active:
            return

        # 자동모드 ON일 때만 엣지 큐 적재
        if rising and self._edge_capture_enabled:
            ts = time.time()
            with self._edge_lock:
                self._edge_events.append(ts)
                if len(self._edge_events) > self._edge_events_max:
                    self._edge_events.popleft()

    def _on_delay_fire(self):
        # (레거시: 사용 안 함)
        pass

    # 가상 센서(펄스)
    def force_sensor_pulse(self, duration_s: float = 0.2):
        now = time.time()
        self._sensor_force_value = 1
        self._sensor_force_until = now + max(0.01, float(duration_s or 0.2))
        self.gui_set_status(f"가상 센서 펄스: {duration_s:.2f}s")

        # ★ 추가: 자동모드 ON & 엣지 수집중이면, 가상 '상승엣지'를 엣지 큐에 주입
        if self._auto_mode_active and self._edge_capture_enabled:
            with self._edge_lock:
                self._edge_events.append(time.time())
                if len(self._edge_events) > self._edge_events_max:
                    self._edge_events.popleft()

    def _clear_forced_sensor(self):
        self._sensor_force_value = None
        self._sensor_force_until = 0.0

    def get_sensor_value(self) -> int:
        if self._sensor_force_value is not None:
            if time.time() < self._sensor_force_until:
                return 1
            self._clear_forced_sensor()
        return int(self._sensor_view) if self._sensor_view is not None else 0

    # 수집된 실센서 상승엣지를 모두 꺼내 반환 (컨트롤러가 소비)
    def take_edge_events(self):
        with self._edge_lock:
            if not self._edge_events:
                return []
            out = list(self._edge_events)
            self._edge_events.clear()
        return out

    # ---------- Execution ----------
    def _maybe_run_motion(self, path: str = None, is_realtime: bool = True):
        effective_path = path or self.selected_path
        if not effective_path or not os.path.exists(effective_path):
            self.gui_set_status("모션 파일 미선택/경로 없음")
            return

        # 실행 중(프로세스 모드) 가드
        if self.proc and self.proc.poll() is None:
            self.gui_set_status("실행 중 → 무시")
            return

        # cooldown
        now = self.now_ms()
        if (now - self.last_run_ms) < self.cooldown_ms:
            self.gui_set_status("쿨다운 중 → 무시")
            return

        # 자동모드 중에는 컨트롤러가 주는 실행만 허용
        if self._auto_mode_active and not self._allow_controller_run_once:
            self.gui_set_status("실행 차단: 자동모드 활성 중 (컨트롤러 외 실행 금지)")
            return
        # 허용 토큰은 1회성
        self._allow_controller_run_once = False

        # INLINE MODE
        if self.exec_node is not None:
            if self._busy_inline:
                self.gui_set_status("실행 중 → 무시")
                return
            self._busy_inline = True
            self.last_run_ms = now

            def _do():
                try:
                    # ▶ is_realtime=True → ServoJ, False → MoveJB 청크
                    self.exec_node.load_motion_from_file(effective_path, is_realtime)
                    if is_realtime:
                        self.gui_set_status("실행 완료(ServoJ)")
                    else:
                        self.gui_set_status("실행 완료(MoveJB 청크)")
                except Exception as e:
                    self.gui_set_status(f"실행 실패: {e}")
                finally:
                    self._busy_inline = False
            threading.Thread(target=_do, daemon=True).start()
            return

        # PROCESS MODE
        cmd = self.launch_cmd_template.format(path=effective_path)
        try:
            if not is_realtime:
                # 외부 런처는 청크/스트림 선택 파라미터가 없어 ServoJ로만 동작
                self.get_logger().warning("process 모드에서는 MoveJB 청크 선택을 지원하지 않아 ServoJ로 실행합니다.")
                self.gui_set_status("process 모드: ServoJ로 실행")
            self.proc = subprocess.Popen(cmd, shell=True)
            self.last_run_ms = now
            self.get_logger().info(f"RUN: {cmd}")
            self.gui_set_status("실행 시작(process)")
        except Exception as e:
            self.gui_set_status(f"실행 실패: {e}")

    def _check_proc(self):
        if self.proc and self.proc.poll() is not None:
            rc = self.proc.returncode
            self.proc = None
            self.gui_set_status(f"완료(code={rc}) → 대기 중")

    # --- 내부: 파일에서 waypoints 뽑기 ---
    def _extract_waypoints_and_params(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        # YAML 우선, 실패 시 JSON 시도
        data = None
        try:
            data = yaml.safe_load(text)
        except Exception:
            data = json.loads(text)

        waypoints = None
        params = None

        if isinstance(data, dict):
            if "motions" in data and isinstance(data["motions"], list) and data["motions"]:
                m0 = data["motions"][0]
                waypoints = m0.get("waypoints")
                params = m0.get("parameters")
            elif "waypoints" in data:
                waypoints = data.get("waypoints")
                params = data.get("parameters")

        if not isinstance(waypoints, list) or len(waypoints) == 0:
            raise ValueError("파싱 실패: waypoints를 찾지 못했습니다.")
        return waypoints, params

    def _movejb_add_safe(self, q, sp=None, ac=None, br=None):
        """
        다양한 SDK 시그니처를 순차 시도하여 MoveJB_Add 호출.
        - q: list/tuple length>=6 (조인트 값)
        - sp, ac, br: 선택적 speed/accel/blend (없으면 None)
        반환: (True, None) 성공 / (False, Exception) 실패
        """
        last_exc = None
        try:
            # 0) sanity
            q6 = [float(x) for x in (q[:6] if isinstance(q, (list, tuple)) else q)]
        except Exception as e:
            return False, e

        # 후보 호출 패턴 목록 (가장 안전한/풍부한-인자 패턴부터)
        patterns = []

        # 1) Joint 객체 + full/부분 인자
        if hasattr(cobot, "Joint"):
            try:
                J = cobot.Joint(*q6)
                patterns.append(("obj_full", (J,
                                              float(sp) if sp is not None else None,
                                              float(ac) if ac is not None else None,
                                              float(br) if br is not None else None)))
                if sp is not None and ac is not None:
                    patterns.append(("obj_sp_ac", (J, float(sp), float(ac))))
                if sp is not None:
                    patterns.append(("obj_sp", (J, float(sp))))
                patterns.append(("obj_only", (J,)))
            except Exception:
                pass

        # 2) 스칼라 6개 + full/부분 인자
        patterns.append(("sc_full", tuple(q6) + (float(sp) if sp is not None else None,
                                                float(ac) if ac is not None else None,
                                                float(br) if br is not None else None)))
        if sp is not None and ac is not None:
            patterns.append(("sc_sp_ac", tuple(q6) + (float(sp), float(ac))))
        if sp is not None:
            patterns.append(("sc_sp", tuple(q6) + (float(sp),)))
        patterns.append(("sc_only", tuple(q6)))

        # 3) 리스트 인자 패턴(일부 바인딩은 list 지원)
        patterns.append(("list_full", (list(q6),
                                       float(sp) if sp is not None else None,
                                       float(ac) if ac is not None else None,
                                       float(br) if br is not None else None)))
        patterns.append(("list_only", (list(q6),)))
        # 4) 리스트 한개 인자
        patterns.append(("list", (list(q6),)))
        # 5) 스칼라 + 일부 인자(대응 안될 수도 있음)
        if sp is not None:
            patterns.append(("sc_sp", tuple(q6) + (float(sp),)))
        if sp is not None and ac is not None:
            patterns.append(("sc_sp_ac", tuple(q6) + (float(sp), float(ac))))

        for name, args in patterns:
            try:
                # 필터: None 인자는 넘기지 않음 (확장 바인딩이 None을 싫어함)
                call_args = tuple(a for a in args if a is not None)
                cobot.MoveJB_Add(*call_args)
                return True, None
            except TypeError as te:
                # 시그니처 불일치: 다음 패턴 시도
                last_exc = te
                continue
            except Exception as e:
                # 바인딩 확장에서 발생한 다른 예외: 보관 후 다음 시도
                last_exc = e
                continue

        # 모든 패턴 실패
        return False, (last_exc or RuntimeError("MoveJB_Add: unknown failure"))

    def run_movejb_servoj_like_stream_from_file(
        self,
        motion_file: str,
        preload: int = 120,
        feed_delay_ms: int = 2,
        default_blend: float = 0.2,
        min_speed: float = 5.0
    ) -> bool:
        """
        ServoJ 기록(wp_dt 기반) 타이밍을 근사하여 MoveJB를 무정지 스트리밍.
        - Δq_max/wp_dt로 각 세그먼트 속도 합성
        - Preload 후 MoveJB_Run 1회 → 실행 중 Add 연속 공급
        - 작은 blend_radius로 경로 생략 최소화
        """
        try:
            import time
            if not motion_file or not os.path.exists(motion_file):
                self.gui_set_status("모션 파일 경로가 없습니다.")
                return False

            now = self.now_ms()
            if (now - self.last_run_ms) < self.cooldown_ms:
                self.gui_set_status("쿨다운 중 → 무시")
                return False
            if self._auto_mode_active and not self._allow_controller_run_once:
                self.gui_set_status("실행 차단: 자동모드 활성 중 (컨트롤러 외 실행 금지)")
                return False
            if self._busy_inline:
                self.gui_set_status("실행 중 → 무시")
                return False
            self._allow_controller_run_once = False

            waypoints, params = self._extract_waypoints_and_params(motion_file)
            wp_dt = float((params or {}).get("wp_dt", getattr(self, "_unity_wpdt_default", 0.05)))
            vmax = float((params or {}).get("joint_speed_deg_s", 80.0))
            amax = float((params or {}).get("joint_accel_deg_s2", 80.0))
            wp_dt = max(0.01, wp_dt)

            seq = []
            for wp in waypoints:
                q = wp.get("joints") or wp.get("q")
                if isinstance(q, (list, tuple)) and len(q) >= 6:
                    seq.append([float(q[i]) for i in range(6)])
            if len(seq) < 2:
                self.gui_set_status("웨이포인트가 부족합니다(2개 이상).")
                return False

            prof = []
            import numpy as _np, math as _m
            def _corner_speed_cap(prev_q, q, next_q, sp, a_max=amax, safety=0.65, R_hint=default_blend):
                v1 = _np.array(q) - _np.array(prev_q)
                v2 = _np.array(next_q) - _np.array(q)
                n1 = float(_np.linalg.norm(v1)) + 1e-9
                n2 = float(_np.linalg.norm(v2)) + 1e-9
                cosang = float(_np.clip(_np.dot(v1, v2)/(n1*n2), -1.0, 1.0))
                ang_deg = float(_m.degrees(_m.acos(cosang)))
                if ang_deg >= 18.0:
                    R = max(0.1, float(R_hint))
                    vcap = max(8.0, (a_max * R) ** 0.5 * safety)
                    return min(sp, vcap)
                return sp

            N = len(seq)
            for i in range(N):
                q = seq[i]
                dq = [(seq[min(i+1, N-1)][k] - seq[max(i-1, 0)][k]) if 0 < i < N-1
                      else (seq[min(i+1, N-1)][k] - seq[i][k]) for k in range(6)]
                dq_abs_max = max(abs(v) for v in dq)
                sp = max(min_speed, min(vmax, dq_abs_max / wp_dt))
                if 0 < i < N-1:
                    sp = _corner_speed_cap(seq[i-1], q, seq[i+1], sp)
                prof.append((q, float(sp), float(amax), float(default_blend)))

            self._busy_inline = True
            self.last_run_ms = now

            def _do():
                try:
                    try:
                        if hasattr(cobot, "MoveJB_Clear"):
                            cobot.MoveJB_Clear()
                    except Exception:
                        pass

                    i = 0
                    preload_n = max(20, min(preload, len(prof)))
                    while i < preload_n:
                        q, sp, ac, br = prof[i]
                        # 기존: 다양한 시그니처 수동 시도 → 제거
                        # try:
                        #     try:
                        #         cobot.MoveJB_Add(q[0], q[1], q[2], q[3], q[4], q[5], sp, ac, br)
                        #     except TypeError:
                        #         if hasattr(cobot, "Joint"):
                        #             cobot.MoveJB_Add(cobot.Joint(*q[:6]))
                        #         else:
                        #             cobot.MoveJB_Add(q[0], q[1], q[2], q[3], q[4], q[5])
                        # except Exception as e:
                        #     self.gui_set_status(f"MoveJB_Add(선적재) 실패: {e}")
                        #     return

                        # 변경: 시그니처 적응형 래퍼 사용
                        success, err = self._movejb_add_safe(q, sp, ac, br)
                        if not success:
                            self.gui_set_status(f"MoveJB_Add(선적재) 실패: {err}")
                            return
                        i += 1

                    try:
                        try:
                            cobot.MoveJB_Run(vmax, amax)
                        except TypeError:
                            try:
                                cobot.MoveJB_Run(amax)
                            except TypeError:
                                try:
                                    cobot.MoveJB_Run(vmax)
                                except Exception:
                                    cobot.MoveJB_Run()
                    except Exception as e:
                        self.gui_set_status(f"MoveJB_Run 실패: {e}")
                        return

                    fd = max(0.0, float(feed_delay_ms) / 1000.0)
                    while i < len(prof):
                        q, sp, ac, br = prof[i]
                        # 기존 수동 시그니처 시도 → 제거 후 래퍼 사용
                        success, err = self._movejb_add_safe(q, sp, ac, br)
                        if not success:
                            self.gui_set_status(f"MoveJB_Add 실패: {err}")
                            return
                        i += 1
                        if fd > 0:
                            time.sleep(fd)

                    self.gui_set_status("수동실행2: MoveJB 스트리밍 완료")
                finally:
                    self._busy_inline = False

            threading.Thread(target=_do, daemon=True).start()
            return True

        except Exception as e:
            self._busy_inline = False
            self.gui_set_status(f"수동실행2 스트리밍 실패: {e}")
            return False

    def run_step_motion(self, src_path: str, step_idx: int) -> bool:
        """선택한 YAML에서 특정 스텝(조인트 1점)을 MoveJB로 실행. 홈/대기와 동일한 파이프라인."""
        try:
            import os, tempfile
            import yaml

            if not src_path or not os.path.exists(src_path):
                self.gui_set_status("선택된 모션 파일이 없습니다.")
                return False

            with open(src_path, "r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            motions = (doc or {}).get("motions") or []
            waypoints = []
            params = {}
            for m in motions:
                if isinstance(m, dict):
                    if not params:
                        params = m.get("parameters") or {}
                    waypoints.extend(m.get("waypoints") or [])

            n = len(waypoints)
            if n == 0:
                self.gui_set_status("웨이포인트가 없습니다.")
                return False
            if step_idx < 0 or step_idx >= n:
                self.gui_set_status(f"잘못된 ID: {step_idx} (0~{n-1})")
                return False

            wp = waypoints[step_idx]
            wptype = str(wp.get("type", "joint")).lower()

            # 1) 조인트 스텝이면 MoveJB로 1점 실행
            if wptype == "joint" or ("joints" in wp):
                q = wp.get("joints") or []
                if not isinstance(q, (list, tuple)) or len(q) < 6:
                    self.gui_set_status("조인트 포맷 오류(6축 필요)")
                    return False

                sp = float(wp.get("speed", params.get("joint_speed_deg_s", 20.0)))
                ac = float(wp.get("accel", params.get("joint_accel_deg_s2", 40.0)))

                # Clear → Add → Run
                try:
                    if hasattr(cobot, "MoveJB_Clear"):
                        cobot.MoveJB_Clear()
                except Exception:
                    pass

                try:
                    if hasattr(cobot, "Joint"):
                        j = cobot.Joint(*q[:6])
                        try:
                            cobot.MoveJB_Add(j)  # SDK에 따라 객체/스칼라 인자 상이
                        except TypeError:
                            cobot.MoveJB_Add(j.j0, j.j1, j.j2, j.j3, j.j4, j.j5, sp, ac, 0.0)
                    else:
                        cobot.MoveJB_Add(q[0], q[1], q[2], q[3], q[4], q[5], sp, ac, 0.0)
                except Exception as e:
                    self.gui_set_status(f"MoveJB_Add 실패: {e}")
                    return False

                try:
                    # 다양한 시그니처 대응
                    try:
                        cobot.MoveJB_Run(sp, ac)
                    except TypeError:
                        try:
                            cobot.MoveJB_Run(ac)
                        except TypeError:
                            try:
                                cobot.MoveJB_Run(sp)
                            except Exception:
                                cobot.MoveJB_Run()
                except Exception as e:
                    self.gui_set_status(f"MoveJB_Run 실패: {e}")
                    return False

                self.gui_set_status(f"스텝 MoveJB 실행 완료(ID={step_idx}, speed={sp}, accel={ac})")
                return True

            # 2) 조인트가 아닌 스텝이면 기존 경로(임시 YAML → ServoJ/프로세스)로 폴백
            tmp = tempfile.NamedTemporaryFile(prefix="rb_step_", suffix=".yaml", delete=False)
            tmp_path = tmp.name
            tmp.close()
            yaml.safe_dump(
                {"motions": [{"waypoints": [wp], "parameters": (params or {})}]},
                open(tmp_path, "w", encoding="utf-8"),
                allow_unicode=True, sort_keys=False,
            )
            self.gui_set_status(f"조인트 아님 → 기존 실행 경로로 수행(ID={step_idx})")
            self._maybe_run_motion(path=tmp_path)
            return True

        except Exception as e:
            self.gui_set_status(f"스텝 실행 실패: {e}")
            return False

    # ---------- Home ----------
    def run_home(self) -> bool:
        # 실행 중(프로세스 모드) 가드
        if self.proc and self.proc.poll() is not None:
            self.proc = None
        if self.proc and self.proc.poll() is None:
            self.gui_set_status("실행 중 → 무시")
            return False

        # cooldown
        now = self.now_ms()
        if (now - self.last_run_ms) < self.cooldown_ms:
            self.gui_set_status("쿨다운 중 → 무시")
            return False

        # 파일 우선: 자동 탐색까지 반영된 _resolved_home_yaml을 먼저 사용
        home_yaml_path = (getattr(self, "_resolved_home_yaml", "") or "").strip()
        if not home_yaml_path:
            home_yaml_path = (self.home_yaml or "").strip()

        resolved = self._resolve_motion_path_safe(home_yaml_path)

        # ✅ 파일 없으면 임시 YAML 생성하지 않고 실패 처리(정책: 홈은 파일 기반 only)
        if not resolved:
            ws = _guess_workspace_root()
            msg = (
                "홈 이동 실패: home_motion.yaml을 찾지 못했습니다.\n"
                f"- 현재 home_yaml 파라미터: {self.home_yaml!r}\n"
                f"- 자동 탐색 결과(_resolved_home_yaml): {getattr(self, '_resolved_home_yaml', '')!r}\n"
                f"- 권장 위치: {ws}/motions/home_motion.yaml\n"
                "해결: (1) motions/home_motion.yaml 생성 또는 "
                "(2) launch/CLI로 -p home_yaml:=<경로> 지정"
            )
            self.gui_set_status("홈 이동 실패: home_motion.yaml 없음 (경로 설정 필요)")
            try:
                self.get_logger().error(msg)
            except Exception:
                pass
            return False

        # --- resolved 파일 실행(기존 로직 유지) ---
        if self.exec_node is not None:
            if self._busy_inline:
                self.gui_set_status("실행 중 → 무시")
                return False
            self._busy_inline = True
            self.last_run_ms = now

            def _do():
                try:
                    self.exec_node.load_motion_from_file(resolved, False)
                    self.gui_set_status("홈 이동 실행(Inline)")
                    self.get_logger().info(f"[HOME] inline run: {resolved}")
                except Exception as e:
                    self.gui_set_status(f"홈 이동 실패: {e}")
                    self.get_logger().error(f"[HOME] inline failed: {e}")
                finally:
                    self._busy_inline = False
            threading.Thread(target=_do, daemon=True).start()
            return True

        cmd = self.launch_cmd_template.format(path=resolved)
        try:
            self.proc = subprocess.Popen(cmd, shell=True)
            self.last_run_ms = now
            self.gui_set_status("홈 이동 실행(process)")
            self.get_logger().info(f"[HOME] run file: {resolved}")
            return True
        except Exception as e:
            self.gui_set_status(f"홈 이동 실패: {e}")
            self.get_logger().error(f"[HOME] run failed: {e}")
            return False
            

    def _resolve_motion_path_safe(self, path: str):
        if not path:
            return None
        path = os.path.expanduser(path)
        if os.path.isfile(path):
            return path
        root, _ = os.path.splitext(path)
        for alt in (root + ".json", root + ".yaml", root + ".yml"):
            if os.path.isfile(alt):
                return alt
        return None
    
    def set_servo_overrides(self, t1=None, t2=None, gain=None, alpha=None):
        if t1 is not None: self.servo_ui['t1'] = float(t1)
        if t2 is not None: self.servo_ui['t2'] = float(t2)
        if gain is not None: self.servo_ui['gain'] = float(gain)
        if alpha is not None: self.servo_ui['alpha'] = float(alpha)
        # 브리지 재생 기본값도 함께 갱신(로그/실행값 일치)
        if t1 is not None: self._unity_play_t1 = float(t1)
        if t2 is not None: self._unity_play_t2 = float(t2)
        if gain is not None: self._unity_play_gain = float(gain)
        if alpha is not None: self._unity_play_alpha = float(alpha)
        # 상태 로그
        try:
            self.get_logger().info(f"[ServoJ OVERRIDE] {self.servo_ui}")
        except Exception:
            pass
        # Inline 모드면 즉시 MotionExecutor에 전달
        if getattr(self, 'exec_node', None):
            try:
                self.exec_node.set_servo_overrides(
                    t1=self.servo_ui['t1'], t2=self.servo_ui['t2'],
                    gain=self.servo_ui['gain'], alpha=self.servo_ui['alpha']
                )
            except Exception as e:
                self.get_logger().warning(f"exec_node override set failed: {e}")

        # Unity 브리지에도 항상 동기화 (exec_node 유무와 무관)
        try:
            if getattr(self, 'unity_bridge', None):
                self.unity_bridge.set_servo_overrides(
                    t1=self.servo_ui['t1'], t2=self.servo_ui['t2'],
                    gain=self.servo_ui['gain'], alpha=self.servo_ui['alpha']
                )
        except Exception as e:
            self.get_logger().warning(f"unity_bridge override set failed: {e}")   
    # ---- Unity Live 헬퍼(API) ----
    def unity_toggle_forward(self, on: bool):
        try:
            self.unity_bridge.set_forwarding(bool(on))
            return True
        except Exception as e:
            self.gui_set_status(f"Unity 포워딩 실패: {e}")
            return False

    def unity_rec_start(self):
        try:
            os.makedirs(self.unity_save_dir, exist_ok=True)
            self.unity_bridge.start_record()
            return True
        except Exception as e:
            self.gui_set_status(f"Unity 저장 시작 실패: {e}")
            return False

    def unity_rec_stop_and_save(self, wp_dt: float = 0.1) -> str:
        try:
            return self.unity_bridge.stop_record_and_save(self.unity_save_dir, wp_dt=float(wp_dt)) or ""
        except Exception as e:
            self.gui_set_status(f"Unity 저장 실패: {e}")
            return ""

    def unity_latest_joint(self):
        try:
            return self.unity_bridge.get_latest_joint()
        except Exception:
            return None

    def unity_record_count(self) -> int:
        try:
            return int(self.unity_bridge.get_recording_count())
        except Exception:
            return 0

    # ---- 원본(파일) 파라미터로 실행 API ----
    def run_motion_file_with_file_params(self, motion_file: str) -> bool:
        """GUI 오버라이드를 잠시 비활성화하고 YAML/JSON의 파라미터 그대로 실행"""
        self._allow_controller_run_once = True
        try:
            if not motion_file or not os.path.exists(motion_file):
                self.gui_set_status("모션 파일 경로가 없습니다.")
                return False
            self.set_selected_path(motion_file)
            if self.exec_node is None:
                self.gui_set_status("inline 모드에서만 지원합니다.")
                return False
            if self._busy_inline:
                self.gui_set_status("실행 중 → 무시")
                return False
            self._busy_inline = True
            self.last_run_ms = self.now_ms()
            def _do():
                try:
                    # 오버라이드 비활성 → 실행 → 복원
                    was = self.exec_node.get_override_enabled()
                    self.exec_node.set_override_enabled(False)
                    self.exec_node.load_motion_from_file(motion_file, True, play_mode_override='interp')
                    self.gui_set_status("실행 완료(원본 파라미터)")
                except Exception as e:
                    self.gui_set_status(f"실행 실패: {e}")
                finally:
                    try:
                        self.exec_node.set_override_enabled(True)
                    except Exception:
                        pass
                    self._busy_inline = False
            threading.Thread(target=_do, daemon=True).start()
            return True
        except Exception as e:
            self.gui_set_status(f"실행 실패: {e}")
            return False
        
    def _servoj_ros_param_cli(self) -> str:
        """Process 모드에서 외부 실행 시 전달할 CLI 파라미터 문자열 생성."""
        p = self.servo_ui
        return (f" --ros-args"
                f" -p servo_t1:={p['t1']:.6f}"
                f" -p servo_t2:={p['t2']:.6f}"
                f" -p servo_gain:={p['gain']:.6f}"
                f" -p servo_alpha:={p['alpha']:.6f}")

    def run_standby(self) -> bool:
        # 실행 중(프로세스 모드) 가드
        if self.proc and self.proc.poll() is not None:
            self.proc = None
        if self.proc and self.proc.poll() is None:
            self.gui_set_status("실행 중 → 무시")
            return False

        # cooldown
        now = self.now_ms()
        if (now - self.last_run_ms) < self.cooldown_ms:
            self.gui_set_status("쿨다운 중 → 무시")
            return False

        # 1) 사용자가 GUI에서 선택해 둔 모션 파일(self.selected_path)을 우선 사용
        preferred = None
        if getattr(self, 'selected_path', None):
            preferred = self._resolve_motion_path_safe(self.selected_path)

        tmp_from_file = None
        if preferred:
            # 선택된 모션 파일에서 '마지막 조인트 웨이포인트'만 추출하여 임시 YAML을 생성한다.
            try:
                with open(preferred, 'r', encoding='utf-8') as f:
                    if preferred.lower().endswith(('.yaml', '.yml')):
                        doc = yaml.safe_load(f)
                    else:
                        doc = json.load(f)
                motions = doc.get('motions') if isinstance(doc, dict) else None
                last_q = None
                last_speed = None
                last_accel = None
                if motions and isinstance(motions, list):
                    for mo in motions:
                        params = mo.get('parameters', {}) or {}
                        for wp in (mo.get('waypoints') or []):
                            wptype = str(wp.get('type', 'cartesian')).lower()
                            if wptype == 'joint' or ('joints' in wp):
                                q = wp.get('joints', [])
                                # 기본 속도/가속도 추출(없으면 파라미터/스탠바이 기본 사용)
                                sp = wp.get('speed', params.get('joint_speed_deg_s', self.standby_speed))
                                ac = wp.get('accel', params.get('joint_accel_deg_s2', self.standby_accel))
                                if isinstance(q, (list, tuple)) and len(q) >= 6:
                                    last_q = [float(x) for x in q[:6]]
                                    last_speed = float(sp)
                                    last_accel = float(ac)
                if last_q is not None:
                    tmp = tempfile.NamedTemporaryFile(prefix="rb_standby_from_selected_", suffix=".yaml", delete=False)
                    tmp_from_file = tmp.name
                    tmp.close()
                    single = {
                        "motions": [
                            {
                                "waypoints": [
                                    {"type": "joint", "joints": last_q, "speed": last_speed}
                                ],
                                "parameters": {
                                    "joint_speed_deg_s": last_speed,
                                    "joint_accel_deg_s2": last_accel,
                                },
                            }
                        ]
                    }
                    with open(tmp_from_file, "w", encoding="utf-8") as wf:
                        yaml.safe_dump(single, wf, allow_unicode=True, sort_keys=False)
                    self.get_logger().info(
                        f"[STANDBY] 선택 모션 마지막 조인트로 이동: {last_q} (sp={last_speed}, ac={last_accel}) | src={preferred}"
                    )
                else:
                    self.get_logger().warning(
                        "[STANDBY] 선택 파일에서 조인트 웨이포인트 없음 → 스탠바이 기본 포즈로 대체"
                    )
            except Exception as e:
                self.get_logger().error(f"[STANDBY] 선택 파일 파싱 실패 → 기본 포즈로 대체: {e}")

            if tmp_from_file:
                if self.exec_node is not None:
                    if self._busy_inline:
                        self.gui_set_status("실행 중 → 무시")
                        return False
                    self._busy_inline = True
                    self.last_run_ms = now

                    def _do():
                        try:
                            self.exec_node.load_motion_from_file(tmp_from_file, False)
                            self.gui_set_status("대기 위치 실행(Inline, 선택파일 마지막 포인트)")
                            self.get_logger().info(f"[STANDBY] inline run(last-only): {tmp_from_file}")
                        except Exception as e:
                            self.gui_set_status(f"대기 위치 실패: {e}")
                            self.get_logger().error(f"[STANDBY] inline failed: {e}")
                        finally:
                            self._busy_inline = False

                    threading.Thread(target=_do, daemon=True).start()
                    return True

                cmd = self.launch_cmd_template.format(path=tmp_from_file)
                self.get_logger().info(f"[STANDBY] run file(last-only): {tmp_from_file}")
                try:
                    self.proc = subprocess.Popen(cmd, shell=True)
                    self.last_run_ms = now
                    self.gui_set_status("대기 위치 실행(process, 선택파일 마지막 포인트)")
                    return True
                except Exception as e:
                    self.gui_set_status(f"대기 위치 실패: {e}")
                    self.get_logger().error(f"[STANDBY] run failed: {e}")
                    return False

        # 임시 YAML 생성
        pose = self.standby_pose_arr
        if not pose or len(pose) != 6:
            self.get_logger().error("[STANDBY] invalid standby_pose_arr")
            self.gui_set_status("대기 위치 실패(포즈 오류)")
            return False

        tmp = tempfile.NamedTemporaryFile(prefix="rb_standby_", suffix=".yaml", delete=False)
        tmp_path = tmp.name
        tmp.close()

        doc = {
            "motions": [
                {
                    "waypoints": [
                        {"type": "joint", "joints": [float(x) for x in pose], "speed": float(self.standby_speed)}
                    ],
                    "parameters": {
                        "joint_speed_deg_s": float(self.standby_speed),
                        "joint_accel_deg_s2": float(self.standby_accel)
                    }
                }
            ]
        }
        with open(tmp_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)

        if self.exec_node is not None:
            if self._busy_inline:
                self.gui_set_status("실행 중 → 무시")
                return False
            self._busy_inline = True
            self.last_run_ms = now

            def _do():
                try:
                    self.exec_node.load_motion_from_file(tmp_path,False)
                    self.gui_set_status("대기 위치 실행(Inline, 임시)")
                    self.get_logger().info(f"[STANDBY→YAML inline] {tmp_path}")
                except Exception as e:
                    self.gui_set_status(f"대기 위치 실패: {e}")
                    self.get_logger().error(f"[STANDBY] inline failed: {e}")
                finally:
                    self._busy_inline = False
            threading.Thread(target=_do, daemon=True).start()
            return True

        cmd = self.launch_cmd_template.format(path=tmp_path)
        self.get_logger().info(f"[STANDBY→YAML] {tmp_path}")
        try:
            self.proc = subprocess.Popen(cmd, shell=True)
            self.last_run_ms = now
            self.gui_set_status("대기 위치 실행(process, 임시)")
            return True
        except Exception as e:
            self.gui_set_status(f"대기 위치 실패: {e}")
            self.get_logger().error(f"[STANDBY] run failed: {e}")
            return False


# ---------------- GUI ----------------
class AutoModePanel(QGroupBox):
    def __init__(self, controller: AutoModeController, parent=None):
        super().__init__("자동 모드", parent)
        self.controller = controller

        self.combo = QComboBox()
        self.label_interval = QLabel("-")
        self.label_wait = QLabel("-")
        self.label_duration = QLabel("-")
        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.status_label = QLabel("대기중")

        grid = QGridLayout()
        grid.addWidget(QLabel("프리셋"), 0, 0); grid.addWidget(self.combo, 0, 1, 1, 3)
        grid.addWidget(QLabel("감지주기(s)"), 1, 0); grid.addWidget(self.label_interval, 1, 1)
        grid.addWidget(QLabel("모션전 대기(s)"), 1, 2); grid.addWidget(self.label_wait, 1, 3)
        grid.addWidget(QLabel("모션시간(s)"), 2, 0); grid.addWidget(self.label_duration, 2, 1)
        grid.addWidget(self.btn_start, 3, 2); grid.addWidget(self.btn_stop, 3, 3)
        grid.addWidget(self.status_label, 4, 0, 1, 4)
        self.setLayout(grid)

        self.btn_start.clicked.connect(self.controller.start)
        self.btn_stop.clicked.connect(self.controller.stop)
        self.combo.currentIndexChanged.connect(self._on_preset_change)
        self.controller.status_signal.connect(self.status_label.setText)
        # running_signal은 Window에서 on/off 토글용으로도 씀

        self._presets = []
        self._preset_by_name = {}

    def load_rules_file(self, yaml_path: str):
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        self._presets = data.get("presets", [])
        self._preset_by_name = {p["name"]: p for p in self._presets}

        self.combo.clear()
        for p in self._presets:
            self.combo.addItem(p["name"])
        if self._presets:
            self._apply_preset(self._presets[0])

    def _on_preset_change(self, idx):
        if 0 <= idx < len(self._presets):
            self._apply_preset(self._presets[idx])

    def _apply_preset(self, preset: dict):
        self.label_interval.setText(str(preset.get("expected_interval_s", "-")))
        self.label_wait.setText(str(preset.get("wait_before_s", "-")))
        self.label_duration.setText(str(preset.get("motion_duration_s", "-")))
        self.controller.load_preset(preset)

    def set_running(self, running: bool):
        self.btn_start.setEnabled(not running)
        self.combo.setEnabled(not running)
        self.btn_stop.setEnabled(running)


class Window(QWidget):
    def __init__(self, node: RunnerNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("RB Motion GUI Runner")
        self.resize(760, 560)  # 초기 창 크기 약간 확대

        # 큰 배너
        self.lbl_big = QLabel("Sensor: 0")
        self.lbl_big.setAlignment(Qt.AlignCenter)
        f = QFont()
        f.setPointSize(40)
        f.setBold(True)
        self.lbl_big.setFont(f)
        self.lbl_big.setStyleSheet("background:#e6e6e6; border-radius:8px; padding:8px;")

        self.lbl_sel = QLabel("선택된 파일: (없음)")
        self.lbl_state = QLabel("센서: -")
        self.lbl_interval = QLabel("고리 간격: -초")
        self.lbl_status = QLabel("상태: 대기 중")
        # 선택 파일 파싱 캐시(경로→(mtime, count))
        self._step_cache = {}

        self.btn_sel = QPushButton("모션 선택"); self.btn_sel.clicked.connect(self._select_yaml)
        self.btn_run = QPushButton("수동 실행"); self.btn_run.clicked.connect(self._manual_run)
        # ▶ 추가: MoveJB 청크 실행 버튼
        self.btn_run2 = QPushButton("수동 실행2(청크)")
        self.btn_run2.setToolTip("MoveJB 청크 방식으로 실행")
        self.btn_run2.clicked.connect(self._manual_run_chunked)
        # ▶ 추가: 원본(파일) 파라미터로 실행 버튼
        self.btn_run_orig = QPushButton("원본 파라미터로 실행")
        self.btn_run_orig.setToolTip("YAML/JSON에 저장된 servo 파라미터 그대로 실행")
        self.btn_run_orig.clicked.connect(self._manual_run_orig_params)
        self.btn_home = QPushButton("홈 이동"); self.btn_home.clicked.connect(self._home)
        self.btn_standby = QPushButton("대기 위치"); self.btn_standby.clicked.connect(self._standby)
        self.btn_estop = QPushButton("비상정지")
        self.btn_estop.setStyleSheet("background:#b22222; color:white; font-weight:bold;")
        self.btn_estop.setToolTip("로봇과 실행 중 작업을 즉시 중단합니다")
        self.btn_estop.clicked.connect(self._emergency_stop)

        # # ⬇⬇⬇ 추가: 최적화 버튼
        # self.btn_opt = QPushButton("최적화 모션(40)")
        # self.btn_opt.setToolTip("선택한 YAML을 ~40 포인트로 줄여 저장 (/home/mgt/Downloads/smart_ws_v2/motions)")
        # self.btn_opt.clicked.connect(self._optimize_motion)

        # ▶ ServoJ 파라미터 그룹
        self.grp_servo = QGroupBox("ServoJ 파라미터(런타임 적용)")
        grid = QGridLayout(self.grp_servo)

        self.sb_t1 = QDoubleSpinBox();  self.sb_t1.setDecimals(4); self.sb_t1.setRange(0.0, 0.1);   self.sb_t1.setSingleStep(0.001)
        self.sb_t2 = QDoubleSpinBox();  self.sb_t2.setDecimals(3); self.sb_t2.setRange(0.0, 0.5);   self.sb_t2.setSingleStep(0.01)
        self.sb_gain = QDoubleSpinBox();self.sb_gain.setDecimals(3); self.sb_gain.setRange(0.0, 0.2); self.sb_gain.setSingleStep(0.005)
        self.sb_alpha = QDoubleSpinBox();self.sb_alpha.setDecimals(4); self.sb_alpha.setRange(0.0, 0.1); self.sb_alpha.setSingleStep(0.001)

        # 초기값(노드에 저장된 값 사용)
        # 브리지 재생에 실제 쓰이는 값(_unity_play_*)을 표시
        self.sb_t1.setValue(getattr(self.node, "_unity_play_t1", self.node.servo_ui.get('t1', 0.05)))
        self.sb_t2.setValue(getattr(self.node, "_unity_play_t2", self.node.servo_ui.get('t2', 0.05)))
        self.sb_gain.setValue(getattr(self.node, "_unity_play_gain", self.node.servo_ui.get('gain', 0.05)))
        self.sb_alpha.setValue(getattr(self.node, "_unity_play_alpha", self.node.servo_ui.get('alpha', 0.004)))

        grid.addWidget(QLabel("t1 (s)"),    0, 0); grid.addWidget(self.sb_t1,   0, 1)
        grid.addWidget(QLabel("t2 (s)"),    0, 2); grid.addWidget(self.sb_t2,   0, 3)
        grid.addWidget(QLabel("gain"),      1, 0); grid.addWidget(self.sb_gain, 1, 1)
        grid.addWidget(QLabel("alpha"),     1, 2); grid.addWidget(self.sb_alpha,1, 3)

        # 값 변경 시 즉시 런타임 반영
        def _apply_servoj():
            self.node.set_servo_overrides(
                t1=self.sb_t1.value(),
                t2=self.sb_t2.value(),
                gain=self.sb_gain.value(),
                alpha=self.sb_alpha.value()
            )
        self.sb_t1.valueChanged.connect(_apply_servoj)
        self.sb_t2.valueChanged.connect(_apply_servoj)
        self.sb_gain.valueChanged.connect(_apply_servoj)
        self.sb_alpha.valueChanged.connect(_apply_servoj)

        self.spin_delay = QDoubleSpinBox()
        self.spin_delay.setDecimals(1)
        self.spin_delay.setSingleStep(0.1)
        self.spin_delay.setRange(0.0, 3600.0)
        self.spin_delay.setSuffix(" s")
        try:
            self.spin_delay.setValue(self.node.post_clear_delay_ms / 1000.0)
        except Exception:
            self.spin_delay.setValue(0.0)
        self.spin_delay.valueChanged.connect(self._on_delay_changed)

        # ---- Unity Live 패널 ----
        # (내부 import 제거: QCheckBox는 상단에서 이미 import)
        self.grp_unity = QGroupBox("Unity Live")
        grid_u = QGridLayout(self.grp_unity)
        self.lbl_u_j = QLabel("최근 Joints(deg): (없음)")
        self.lbl_u_cnt = QLabel("기록 샘플: 0")
        self.lbl_u_file = QLabel("파일: (없음)")
        self.lbl_u_status = QLabel("상태: 대기")
        self.chk_u_forward = QCheckBox("라이브 포워딩(즉시 송신)")
        self.chk_u_forward.setChecked(True)
        self.chk_u_forward.toggled.connect(self._on_unity_forward)
        self.btn_u_rec_start = QPushButton("Unity 저장 시작")
        self.btn_u_rec_stop  = QPushButton("Unity 저장 정지(파일 저장)")
        self.btn_u_rec_start.clicked.connect(self._on_unity_rec_start)
        self.btn_u_rec_stop.clicked.connect(self._on_unity_rec_stop)
        grid_u.addWidget(self.chk_u_forward, 0, 0, 1, 2)
        grid_u.addWidget(self.lbl_u_j,       1, 0, 1, 2)
        grid_u.addWidget(self.lbl_u_cnt,     2, 0, 1, 2)
        grid_u.addWidget(self.lbl_u_file,    3, 0, 1, 2)
        grid_u.addWidget(self.lbl_u_status,  4, 0, 1, 2) 
        # 버튼 표시 추가
        grid_u.addWidget(self.btn_u_rec_start, 5, 0, 1, 1)
        grid_u.addWidget(self.btn_u_rec_stop,  5, 1, 1, 1)       

        top = QHBoxLayout()
        top.addWidget(self.btn_sel)
        top.addWidget(self.btn_run)
        top.addWidget(self.btn_run2)  
        top.addWidget(self.btn_run_orig)
        top.addWidget(self.btn_home)
        #top.addWidget(self.btn_opt)  
        top.addWidget(self.btn_estop)

        delay_row = QHBoxLayout()
        delay_row.addStretch(1)
        delay_row.addWidget(QLabel("시작 지연(초):"))
        delay_row.addWidget(self.spin_delay)
        delay_row.addStretch(1)

        row2 = QHBoxLayout()
        row2.addStretch(1)
        row2.addWidget(self.btn_standby)
        row2.addStretch(1)

        lay = QVBoxLayout(self)
        lay.addWidget(self.lbl_big)
        lay.addLayout(top)
        lay.addLayout(delay_row)
        lay.addWidget(self.grp_unity)

        # ▶ 여기서 파라미터 그룹을 레이아웃에 추가해야 화면에 보입니다.
        # lay.addWidget(self.grp_servo)

        lay.addLayout(row2)
        lay.addWidget(self.lbl_sel)
        lay.addWidget(self.lbl_state)
        lay.addWidget(self.lbl_interval)
        lay.addWidget(self.lbl_status)

        # 자동 모드 컨트롤러/패널
        self.auto_ctrl = AutoModeController(
            self.node.run_motion_file,
            self.node.get_sensor_value,
            edge_events_fn=self.node.take_edge_events,
            is_busy_fn=self.node.is_motion_busy,
        )

        self.auto_panel = AutoModePanel(self.auto_ctrl)
        # ✅ auto_rules.yaml 경로도 default_dir 기반으로
        self.rules_path = str(Path(self.node.default_dir) / "auto_rules.yaml")
        try:
            if os.path.exists(self.rules_path):
                self.auto_panel.load_rules_file(self.rules_path)
            else:
                self.set_status("auto_rules.yaml 없음(자동모드 패널은 파일 로드 전용)")
        except Exception as e:
            self.set_status(f"오토 룰스 로드 실패: {e}")
        
        # ▶ 수동 새로고침 버튼
        rules_row = QHBoxLayout()
        btn_rules_reload = QPushButton("오토 룰스 새로고침")
        btn_rules_reload.setToolTip("auto_rules.yaml 변경 후 최신 내용으로 다시 불러옵니다.")
        btn_rules_reload.clicked.connect(self._reload_rules)
        rules_row.addWidget(btn_rules_reload)
        rules_row.addStretch(1)
        lay.addLayout(rules_row)

        lay.addWidget(self.auto_panel)

        # 자동모드 on/off → RunnerNode에 수집/차단 지시
        self.auto_ctrl.running_signal.connect(self._on_auto_running)
        self.auto_ctrl.running_signal.connect(self.auto_panel.set_running)

       
        # --- 스텝 모션 UI ---
        self.step_group = QGroupBox("스텝 모션")
        sg2 = QGridLayout()

        self.step_id = QSpinBox()
        self.step_id.setRange(0, 9999)    # 파일 로드시 갱신해도 좋지만, 간단히 큰 범위 허용
        self.step_id.setValue(0)

        self.btn_step_run = QPushButton("선택 ID 1점 실행")

        sg2.addWidget(QLabel("ID (0-based)"), 0, 0)
        sg2.addWidget(self.step_id,            0, 1)
        sg2.addWidget(self.btn_step_run,       1, 0, 1, 2)
        self.step_group.setLayout(sg2)
        lay.addWidget(self.step_group)

        # 클릭 연결
        self.btn_step_run.clicked.connect(self._run_step_once)

        # TCP 값 수동 입력 패널 (자동모드에서 허용)
        self.grp_tcp = QGroupBox("TCP 값 수동 입력(테스트)")
        grid_tcp = QGridLayout(self.grp_tcp)
        self.edit_tcp = QLineEdit()
        self.edit_tcp.setPlaceholderText("예: -2 또는 여러 값: -2, 1 0 none")
        self.btn_tcp_send = QPushButton("값 보내기")
        self.btn_tcp_send.setToolTip("자동모드 프리셋이 tcp_value일 때, 값 큐에 주입합니다.")
        grid_tcp.addWidget(QLabel("값 입력"), 0, 0)
        grid_tcp.addWidget(self.edit_tcp, 0, 1)
        grid_tcp.addWidget(self.btn_tcp_send, 0, 2)
        self.grp_tcp.setLayout(grid_tcp)
        lay.addWidget(self.grp_tcp)
        self.btn_tcp_send.clicked.connect(self._send_tcp_values)

        # 시작 지연은 오토룰스에서만 사용, GUI 지연은 비활성화
        try:
            self.spin_delay.setEnabled(False)
        except Exception:
            pass

    def _set_controls_enabled_for_auto(self, running: bool):
        # 자동모드 running=True일 때 TCP 입력과 비상정지만 활성화
        allow = not running
        for w in [self.btn_sel, self.btn_run, self.btn_run2, self.btn_run_orig,
                  self.btn_home, self.btn_standby,
                  self.btn_step_run, self.chk_u_forward, self.btn_u_rec_start,
                  self.btn_u_rec_stop]:
            try:
                w.setEnabled(allow)
            except Exception:
                pass
        try:
            self.grp_servo.setEnabled(allow)
        except Exception:
            pass
        try:
            self.grp_unity.setEnabled(allow)
        except Exception:
            pass
        # 항상 허용: 비상정지, TCP 입력
        try:
            self.btn_estop.setEnabled(True)
        except Exception:
            pass
        try:
            self.grp_tcp.setEnabled(True)
        except Exception:
            pass

        # UI 폴링
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._pump_gui)
        self._ui_timer.start(50)  # 20Hz
        self._last_sensor_shown = None
        self._last_status_shown = None

        # (위로 이동됨)

    # ---- Unity Live 핸들러 ----
    def _on_unity_forward(self, on: bool):
        ok = self.node.unity_toggle_forward(bool(on))
        self.lbl_u_status.setText(f"상태: 라이브 포워딩 {'ON' if on and ok else 'OFF'}")

    def _on_unity_rec_start(self):
        if self.node.unity_rec_start():
            self.lbl_u_status.setText("상태: 저장 시작")

    def _on_unity_rec_stop(self):
        path = self.node.unity_rec_stop_and_save(wp_dt=0.1)
        if path:
            self.lbl_u_file.setText(f"파일: {path}")
            self.lbl_u_status.setText("상태: 저장 완료")
        else:
            self.lbl_u_status.setText("상태: 저장 실패/데이터 없음")

    def _reload_rules(self):
        try:
            prev = self.auto_panel.combo.currentText()
            self.auto_panel.load_rules_file(self.rules_path)
            if prev:
                idx = self.auto_panel.combo.findText(prev)
                if idx >= 0:
                    # 이전 선택 복원
                    self.auto_panel.combo.blockSignals(True)
                    self.auto_panel.combo.setCurrentIndex(idx)
                    self.auto_panel.combo.blockSignals(False)
                    # 내용 반영
                    self.auto_panel._on_preset_change(idx)
            self.set_status("오토 룰스 새로고침 완료")
        except Exception as e:
            self.set_status(f"룰스 리로드 실패: {e}")

    def _run_step_once(self):
        if not self.node.selected_path or not os.path.exists(self.node.selected_path):
            self.set_status("선택된 모션 파일이 없습니다.")
            return
        idx = int(self.step_id.value())
        ok = self.node.run_step_motion(self.node.selected_path, idx)
        if ok:
            self.set_status(f"스텝 실행: ID={idx}")

    # 자동모드 on/off → 엣지 수집 토글
    def _on_auto_running(self, running: bool):
        self.node.set_edge_capture_enabled(running)
        self._set_controls_enabled_for_auto(running)

    # 가상 센서 버튼
    def _pulse_once(self):
        dur = float(self.spin_pulse.value())
        self.node.force_sensor_pulse(dur)
        self.set_status(f"가상 센서: 1 펄스 {dur:.2f}s")

    def _pump_gui(self):
        v = int(self.node.get_sensor_value())
        if v != self._last_sensor_shown:
            self._last_sensor_shown = v
            self.set_sensor_state(v)

        if getattr(self.node, "_pending_status", None) is not None:
            s = self.node._pending_status
            self.node._pending_status = None
            if s != self._last_status_shown:
                self._last_status_shown = s
                self.set_status(s)

        # 상승엣지 간 간격 펜딩 값 반영
        if getattr(self.node, "_pending_interval_s", None) is not None:
            try:
                val = float(self.node._pending_interval_s)
                self.set_interval(val)
            except Exception:
                pass
            finally:
                self.node._pending_interval_s = None

        # Unity Live 상태는 매 주기 갱신
        try:
            q = self.node.unity_latest_joint() or []
            if q:
                self.lbl_u_j.setText("최근 Joints(deg): " + ", ".join(f"{x:.2f}" for x in q))
            else:
                self.lbl_u_j.setText("최근 Joints(deg): (없음)")
            self.lbl_u_cnt.setText(f"기록 샘플: {self.node.unity_record_count()}")
        except Exception:
            pass         

    # GUI setters
    def set_selected_path(self, path: str):
        if path and os.path.exists(path):
            try:
                base = os.path.splitext(os.path.basename(path))[0]  # 경로·확장자 제거
            except Exception:
                base = path
            self.lbl_sel.setText(f"선택된 파일: {base}")
        else:
            self.lbl_sel.setText("선택된 파일: (없음)")

    def set_status(self, s: str):
        self.lbl_status.setText(f"상태: {s}")

    def set_sensor_state(self, v: int):
        self.lbl_state.setText(f"센서: {v}")
        self.lbl_big.setText(f"Sensor: {v}")

    def set_interval(self, seconds: float):
        """상승엣지 간 간격을 라벨에 표시."""
        try:
            self.lbl_interval.setText(f"고리 간격: {seconds:.2f}초")
        except Exception:
            self.lbl_interval.setText("고리 간격: -초")

    # Buttons
    def _select_yaml(self):
        # 자동모드 활성 시 파일 선택은 비활성
        if getattr(self.node, '_auto_mode_active', False):
            self.set_status("자동모드 활성화: 파일 선택 비활성")
            return
        start_dir = self.node.default_dir if self.node.default_dir else os.path.expanduser('~')
        # 네이티브 파일 다이얼로그가 느릴 수 있어 Qt 비네이티브로 강제
        try:
            dlg = QFileDialog(self, "모션 파일 선택", start_dir, "Motion files (*.yaml *.yml *.json)")
            dlg.setOption(QFileDialog.DontUseNativeDialog, True)
            dlg.setFileMode(QFileDialog.ExistingFile)
            dlg.setViewMode(QFileDialog.Detail)
            dlg.setNameFilter("Motion files (*.yaml *.yml *.json)")
            if dlg.exec_() != QFileDialog.Accepted:
                return
            sel = dlg.selectedFiles() or []
            path = sel[0] if sel else None
        except Exception:
            # 폴백: 기존 정적 API 사용
            path, _ = QFileDialog.getOpenFileName(
                self, "모션 파일 선택",
                start_dir,
                "Motion files (*.yaml *.yml *.json)"
            )
        if not path:
            return
        self.node.set_selected_path(path)
        self.set_status("파일 선택됨")
        self._update_step_range_async(self.node.selected_path)
        # 자동모드에서는 선택만 반영하고 실행하지 않음(상태 메시지)
        if getattr(self.node, '_auto_mode_active', False):
            self.set_status("자동모드 활성화: 선택만 반영 (수동 실행 비활성)")

    def _update_step_range(self, path: str):
        try:
            waypoints, _ = self.node._extract_waypoints_and_params(path)
            n = len(waypoints)
            self.step_id.setRange(0, max(0, n-1))
            self.set_status(f"로드됨: waypoints={n} (ID 0~{n-1})")
        except Exception:
            # 못 읽어도 기존 범위 유지
            pass

    def _update_step_range_async(self, path: str):
        import threading, os
        if not path or not os.path.exists(path):
            return
        try:
            mtime = os.path.getmtime(path)
            cached = self._step_cache.get(path)
            if cached and cached[0] == mtime:
                n = int(cached[1])
                self.step_id.setRange(0, max(0, n-1))
                self.set_status(f"로드됨(캐시): waypoints={n} (ID 0~{n-1})")
                return
        except Exception:
            pass

        # 비동기 파싱(UI 렉 방지)
        self.set_status("파일 분석 중…")
        self.btn_step_run.setEnabled(False)
        def _work():
            n = None
            try:
                wps, _ = self.node._extract_waypoints_and_params(path)
                n = len(wps) if isinstance(wps, list) else 0
            except Exception:
                n = None
            def _finish():
                try:
                    if n is None:
                        self.set_status("파싱 실패: waypoints 읽기 오류")
                    else:
                        import os as _os
                        try:
                            self._step_cache[path] = (_os.path.getmtime(path), int(n))
                        except Exception:
                            self._step_cache[path] = (None, int(n))
                        self.step_id.setRange(0, max(0, int(n)-1))
                        self.set_status(f"로드됨: waypoints={int(n)} (ID 0~{int(n)-1})")
                finally:
                    try:
                        self.btn_step_run.setEnabled(True)
                    except Exception:
                        pass
            QTimer.singleShot(0, _finish)
        threading.Thread(target=_work, daemon=True).start()

    def _on_delay_changed(self, sec):
        try:
            sec = float(sec)
        except Exception:
            sec = self.node.post_clear_delay_ms / 1000.0
        self.node.set_start_delay_seconds(sec)
        self.lbl_status.setText(f"시작 지연: {sec:.1f}초")

    def _manual_run(self):
        if getattr(self.node, '_auto_mode_active', False):
            self.set_status("자동모드 활성화: 수동 실행 비활성")
            return
        self.node._maybe_run_motion_via_bridge()

    def _manual_run_chunked(self):
        """수동 실행2: MoveJB 스트리밍(ServoJ 기록 근사)"""
        # 선택 파일 확인
        if not self.node.selected_path or not os.path.exists(self.node.selected_path):
            self.set_status("선택된 모션 파일이 없습니다.")
            return

        # SDK 호환 확인(최소 MoveJB_Add/Run 존재)
        if not (hasattr(cobot, "MoveJB_Add") and hasattr(cobot, "MoveJB_Run")):
            self.set_status("SDK가 MoveJB 스트리밍을 지원하지 않습니다(함수 없음).")
            return
        if getattr(self.node, '_auto_mode_active', False):
            self.set_status("자동모드 활성화: 수동 실행 비활성")
            return
        ok = self.node.run_movejb_servoj_like_stream_from_file(
            self.node.selected_path,
            preload=100,
            feed_delay_ms=2,
            default_blend=0.2,
            min_speed=5.0
        )
        if ok:
            self.set_status("수동 실행2: MoveJB 스트리밍 시작")
        else:
            self.set_status("수동 실행2: 시작 실패")

    def _manual_run_orig_params(self):
        """YAML/JSON 파일에 저장된 파라미터 그대로 실행(오버라이드 무시)"""
        if not self.node.selected_path or not os.path.exists(self.node.selected_path):
            self.set_status("선택된 모션 파일이 없습니다.")
            return
        if getattr(self.node, '_auto_mode_active', False):
            self.set_status("자동모드 활성화: 수동 실행 비활성")
            return
        self.node.run_motion_file_with_file_params(self.node.selected_path)

    def _home(self):
        self.node.run_home()

    def _standby(self):
        self.node.run_standby()

    def _emergency_stop(self):
        self.node.emergency_stop()

    def _send_tcp_values(self):
        """입력칸의 토큰을 값 큐에 순서대로 주입. 정수 또는 'none' 허용."""
        text = (self.edit_tcp.text() or "").strip()
        if not text:
            self.set_status("값 입력 없음")
            return
        toks = []
        for tok in text.replace(",", " ").split():
            if tok.strip().lower() == "none":
                toks.append(None)
            else:
                try:
                    toks.append(int(tok))
                except Exception:
                    pass
        if not toks:
            self.set_status("정수/none 파싱 실패")
            return
        for v in toks:
            try:
                if hasattr(self, "auto_ctrl"):
                    self.auto_ctrl.feed_value(v)
            except Exception:
                pass
        disp = ", ".join(("none" if v is None else str(v)) for v in toks)
        self.set_status(f"수동 값 주입: {disp}")
        self.edit_tcp.clear()


def main(args=None):
    """ROS2 콘솔 스크립트 엔트리포인트.
    Qt 이벤트 루프와 ROS2 executor를 QTimer로 통합하여 GUI와 노드를 함께 구동.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    try:
        rclpy.init(args=args)
    except Exception:
        # 이미 초기화되어 있거나 환경 문제일 때도 실행을 계속 시도
        pass

    exit_code = 0
    node = None
    executor = None
    try:
        node = RunnerNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Qt 타이머로 ROS 스핀을 주기적으로 수행
        qt_timer = QTimer()
        def _spin_once():
            try:
                executor.spin_once(timeout_sec=0.01)
            except Exception:
                pass
        qt_timer.timeout.connect(_spin_once)
        qt_timer.start(10)  # 10ms 주기

        # GUI 표시 및 실행
        try:
            node.win.show()
        except Exception:
            pass
        exit_code = node.app.exec_()

    finally:
        try:
            if executor and node:
                executor.remove_node(node)
        except Exception:
            pass
        try:
            if node:
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    return exit_code

if __name__ == '__main__':
    import sys
    sys.exit(main())
