#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MotionExecutor (servo-streaming only for joint paths)

This version was simplified to keep only the ServoJ-based streaming runner
(_run_joint_path_chunked2) for joint motions and remove the previous
chunking/MoveJB batch logic per user request.
"""

import os
import json
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Int32, Float32

# Optional YAML support
try:
    import yaml
except Exception:
    yaml = None

from rb_test import cobot


# ---------- utils ----------
def _as_float(x, default=0.0) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def _as_list6(x) -> List[float]:
    """Accepts list/tuple of length >=6 or dict with j1..j6 keys, returns 6 floats."""
    if isinstance(x, dict):
        keys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        return [_as_float(x.get(k, 0.0)) for k in keys]
    return [_as_float(v) for v in list(x)][:6]


def _get_blend_option(name: str):
    try:
        return getattr(cobot.BLEND_OPTION, str(name).upper())
    except Exception:
        return cobot.BLEND_OPTION.RATIO  # default in cobot.BLEND_OPTION


def _get_blend_rtype(name: str):
    try:
        return getattr(cobot.BLEND_RTYPE, str(name).upper())
    except Exception:
        return cobot.BLEND_RTYPE.INTENDED


class MotionExecutor(Node):
    def __init__(self):
        super().__init__('motion_executor')
        

        # Params
        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('motion_file', '')
        self.declare_parameter('auto_load', False)

        ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self._maybe_connect(ip)

        self.get_logger().info("✅ MotionExecutor 노드 초기화 완료 (servo-streaming mode)")

        # 진행률 퍼블리셔 (선택)
        progress_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.progress_pub = self.create_publisher(Int32, "/motion_executor/wp_progress", progress_qos)
        self.duration_pub = self.create_publisher(Float32, "/motion_executor/last_duration_s", 10)

        self.is_realtime = True  # servoJ 모드 고정
        self._cancel_requested = False  # 긴급 정지 플래그
        self._override_enabled = True # GUI 오버라이드 적용 여부(기본 True)
        # Auto-load
        mf = self.get_parameter('motion_file').get_parameter_value().string_value
        auto = bool(self.get_parameter('auto_load').value)
        if auto and mf:
            try:
                self.load_motion_from_file(mf)
            except Exception as e:
                self.get_logger().error(f"Auto-load failed: {e!r}")

        # ▶ ServoJ 기본 파라미터(노드 파라미터로도 설정 가능)
        self.declare_parameter('servo_t1', 0.05)
        self.declare_parameter('servo_t2', 0.05)
        self.declare_parameter('servo_gain', 0.05)
        self.declare_parameter('servo_alpha', 0.003)
        self.declare_parameter('play_mode', 'hold')

        # ▶ GUI에서 오는 런타임 오버라이드 저장소
        self.servo_overrides: Dict[str, float] = {}
        self.declare_parameter('log_each_wp', True)   # ← 추가: 세그 변경 때 로그
        self._log_each_wp = bool(self.get_parameter('log_each_wp').value)

    def _maybe_connect(self, ip: str):
        try:
            if getattr(cobot, 'CMDSock', None) is None and hasattr(cobot, 'ConnectToCB'):
                self.get_logger().info(f"Connecting to robot controller at {ip} ...")
                cobot.ConnectToCB(ip)
        except Exception as e:
            self.get_logger().warning(f"ConnectToCB failed or not needed: {e}")

    def load_motion_from_file(self, file_path: str, is_realtime: bool = True, play_mode_override: str = None):
        """Load a YAML/JSON motion file and execute (servo-streaming for joint motions)."""
        # 새 실행마다 취소 플래그 초기화
        self._cancel_requested = False
        if not os.path.isfile(file_path):
            self.get_logger().error(f"❌ 파일이 존재하지 않습니다: {file_path}")
            return

        # Parse
        try:
            data = None
            if file_path.endswith(('.yaml', '.yml')):
                import yaml
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
            elif file_path.endswith('.json'):
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            else:
                raise RuntimeError("지원하지 않는 파일 형식(확장자)")
        except Exception as e:
            self.get_logger().error(f"파싱 실패: {e}")
            return

        motions = data.get('motions')
        if not isinstance(motions, list) or not motions:
            self.get_logger().error("❌ motions 리스트가 비었습니다.")
            return

        # Collect waypoints
        joint_wps: List[Dict[str, Any]] = []
        cart_wps: List[Dict[str, Any]] = []
        first_params: Dict[str, Any] = {}

        for mo in motions:
            if not isinstance(mo, dict):
                continue
            params = mo.get('parameters', {}) or {}
            if not first_params:
                first_params = params

            for wp in mo.get('waypoints', []) or []:
                wptype = str(wp.get('type', 'cartesian')).lower()

                if wptype == 'joint' or 'joints' in wp:
                    q = _as_list6(wp.get('joints', []))
                    speed = _as_float(wp.get('speed', params.get('joint_speed_deg_s', 20.0)))
                    accel = _as_float(wp.get('accel', params.get('joint_accel_deg_s2', 40.0)))
                    joint_wps.append({'q': q, 'speed': speed, 'accel': accel})
                else:
                    pose = wp.get('pose', {})
                    pos, ori = pose.get('position', {}), pose.get('orientation', {})
                    try:
                        pose_list = [
                            _as_float(pos['x']), _as_float(pos['y']), _as_float(pos['z']),
                            _as_float(ori['rx']), _as_float(ori['ry']), _as_float(ori['rz']),
                        ]
                    except Exception as e:
                        self.get_logger().error(f"❌ pose 키 누락: {e}")
                        continue

                    speed = _as_float(wp.get('speed', params.get('linear_speed_mm_s', 100.0)))
                    blend = _as_float(wp.get('blend_radius', 5.0))
                    cart_wps.append({'pose': pose_list, 'speed': speed, 'blend': blend})

        if joint_wps:
            self.get_logger().info(f"📊 총 {len(joint_wps)}개 조인트 웨이포인트 처리(servo stream)")
            # 실행 모드 결정: override > 파일 파라미터 > 노드 파라미터
            pm_node = str(self.get_parameter('play_mode').get_parameter_value().string_value or 'hold').lower()
            pm_file = str(first_params.get('play_mode', pm_node)).lower()
            pm = str(play_mode_override or pm_file or pm_node).lower()
            if is_realtime:
                if pm == 'hold':
                    self._run_joint_servoJ_hold(joint_wps, first_params)
                else:
                    self._run_joint_servoJ(joint_wps, first_params)
            else:
                self._run_joint_path_chunked(joint_wps, first_params)
            return

        if cart_wps:
            self.get_logger().info(f"📊 총 {len(cart_wps)}개 카르테시안 웨이포인트 처리")
            opt = _get_blend_option(first_params.get('blend_option', 'RATIO'))
            rtype = _get_blend_rtype(first_params.get('blend_type', 'INTENDED'))
            self._run_cartesian_path(cart_wps, opt, rtype)
            return

        self.get_logger().error("❌ 수집된 웨이포인트가 없습니다.")

    def _run_joint_path_chunked(self, joint_wps: List[Dict[str, Any]], params: Dict[str, Any], chunk_size: int = 40):
        """Run joint path in chunks to handle waypoint limitation."""
        import time
        start_perf = time.perf_counter()

        total_waypoints = len(joint_wps)
        num_chunks = (total_waypoints + chunk_size - 1) // chunk_size
        
        self.get_logger().info(f"🔄 청킹 모드: {total_waypoints}개 → {num_chunks}개 청크 (각 최대 {chunk_size}개)")
        
        for chunk_idx in range(num_chunks):
            if self._cancel_requested:
                self.get_logger().warning("[HALT] 청킹 실행 중단 요청 감지 → MotionHalt 호출")
                try:
                    cobot.MotionHalt()
                except Exception:
                    pass
                break
            start_idx = chunk_idx * chunk_size
            end_idx = min(start_idx + chunk_size, total_waypoints)
            chunk_wps = joint_wps[start_idx:end_idx]
            
            self.get_logger().info(f"📦 청크 {chunk_idx+1}/{num_chunks}: {len(chunk_wps)}개 웨이포인트")
            
            # Clear buffer for this chunk
            if hasattr(cobot, 'MoveJB_Clear'):
                cobot.MoveJB_Clear()
            
            # Add waypoints for this chunk
            added = 0
            for m in chunk_wps:
                q = m['q']
                j = cobot.Joint(*q)
                try:
                    cobot.MoveJB_Add(j)
                except TypeError:
                    cobot.MoveJB_Add(*q)
                added += 1
                try:
                    self.progress_pub.publish(Int32(data=added + start_idx))
                except Exception:
                    pass
            
            self.get_logger().info(f"  └─ {added}개 추가 완료")
            
            # Run this chunk
            sp = _as_float(params.get('joint_speed_deg_s', chunk_wps[0]['speed']))
            ac = _as_float(params.get('joint_accel_deg_s2', chunk_wps[0]['accel']))
            
            try:
                cobot.MoveJB_Run(sp, ac)
            except TypeError:
                try:
                    cobot.MoveJB_Run(sp)
                except Exception:
                    cobot.MoveJB_Run(ac)
            
            self.get_logger().info(f"  └─ 청크 {chunk_idx+1} 실행 중...")
            
            # Wait for chunk to complete (except last chunk)
            if chunk_idx < num_chunks - 1:
                # 취소 요청 확인
                if self._cancel_requested:
                    self.get_logger().warning("[HALT] 청크 대기 중 중단 요청 감지")
                    try:
                        cobot.MotionHalt()
                    except Exception:
                        pass
                    break
                time.sleep(0.2)  # Increased delay between chunks for stability
        
        self.get_logger().info(f"✅ 청킹 모드 완료! 총 {total_waypoints}개 웨이포인트 실행")
        try:
            elapsed = time.perf_counter() - start_perf
            self._publish_duration(elapsed)
        except Exception:
            pass

    def set_servo_overrides(self, t1=None, t2=None, gain=None, alpha=None):
        """GUI가 실시간으로 세팅하는 ServoJ 오버라이드 값."""
        ov = {}
        if t1 is not None: ov['t1'] = float(t1)
        if t2 is not None: ov['t2'] = float(t2)
        if gain is not None: ov['gain'] = float(gain)
        if alpha is not None: ov['alpha'] = float(alpha)
        self.servo_overrides.update(ov)
        self.get_logger().info(f"[ServoJ OVERRIDE] {self.servo_overrides}")

    def _publish_duration(self, seconds: float):
        try:
            self.duration_pub.publish(Float32(data=float(seconds)))
        except Exception:
            pass

    def set_override_enabled(self, enabled: bool):
        """GUI 오버라이드(servo_t1/t2/gain/alpha)를 적용할지 여부"""
        self._override_enabled = bool(enabled)

    def get_override_enabled(self) -> bool:
        return bool(self._override_enabled)

    def _run_joint_servoJ(self, joint_wps: List[Dict[str, Any]], params: Dict[str, Any], chunk_size: int = 40):
        """고주기(예: 8ms)로 선형 보간하여 ServoJ 스트리밍."""
        total_waypoints = len(joint_wps)
        if total_waypoints < 1:
            self.get_logger().error("❌ 조인트 웨이포인트가 없습니다.")
            return

        # 1) 파일 params → 2) 노드 파라미터 → 3) GUI 오버라이드 순으로 병합
        t1 = float(params.get('servo_t1', self.get_parameter('servo_t1').value))
        t2 = float(params.get('servo_t2', self.get_parameter('servo_t2').value))
        gain = float(params.get('servo_gain', self.get_parameter('servo_gain').value))
        servo_alpha = float(params.get('servo_alpha', self.get_parameter('servo_alpha').value))
        # 't1': 0.008, 't2': 0.05, 'gain': 0.05, 'alpha': 0.005 처음 값
        # 't1': 0.08, 't2': 0.05, 'gain': 0.05, 'alpha': 0.004 대표님과 함께 수정한 값

        # GUI 오버라이드 적용은 스위치가 켜졌을 때만
        if getattr(self, '_override_enabled', True):
            ov = getattr(self, 'servo_overrides', {}) or {}
            if 't1' in ov: t1 = float(ov['t1'])
            if 't2' in ov: t2 = float(ov['t2'])
            if 'gain' in ov: gain = float(ov['gain'])
            if 'alpha' in ov: servo_alpha = float(ov['alpha'])

        # 제어 주기와 원본 웨이포인트 간격
        Ts = float(params.get('servo_Ts', 0.008))   # 125Hz
        Ts = max(0.002, min(Ts, 0.05))              # 안전 클램프(유지 권장)
        wp_dt = float(params.get('wp_dt', 0.1))     # 원래 저장 간격(추정: 0.1초)

        # 외부 보간 신호용 LPF는 별도 파라미터로 분리(1.0이면 LPF 비활성)
        filter_alpha = float(params.get('filter_alpha', 1.0))
        filter_alpha = max(0.0, min(filter_alpha, 1.0))

        # 전체 타임라인 스케일(>1.0이면 더 천천히 진행)
        time_scale = float(params.get('time_scale', 1.0))
        time_scale = max(0.1, min(time_scale, 10.0))
        wp_dt_scaled = max(1e-4, wp_dt * time_scale)

        # 너무 작은 t1에서의 급변 억제(안전장치)
        internal_blend = max(t1, 3.0 * Ts)

        def _quintic_blend(s: float) -> float:
            if s <= 0.0: return 0.0
            if s >= 1.0: return 1.0
            return 10.0*s**3 - 15.0*s**4 + 6.0*s**5

        def _lerp(a: float, b: float, s: float) -> float:
            return (1.0 - s) * a + s * b

        def _sleep_until(target: float):
            now = time.perf_counter()
            remain = target - now
            if remain > 0.0015:
                time.sleep(remain - 0.0007)
            while time.perf_counter() < target:
                pass

        # 시간축(가정: 균일 간격 wp_dt_scaled)
        ts = [i * wp_dt_scaled for i in range(total_waypoints)]

        # 시작 로그
        start_wall = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        start_perf = time.perf_counter()
        self.get_logger().info(f"⏱ _run_joint_servoJ(보간) 시작: {start_wall}")
        self.get_logger().info(
            f"▶ Ts={Ts:.4f}s, wp_dt={wp_dt:.3f}s, time_scale={time_scale:.2f}, "
            f"t1={t1}, t2={t2}, gain={gain}, servo_alpha={servo_alpha}, filter_alpha={filter_alpha}"
        )

        # 초기 참조값은 첫 웨이포인트로 시작
        q_ref_prev = list(joint_wps[0]['q'])

        last_seg = -1
        t0 = time.perf_counter()
        k = 0
        while True:
            if self._cancel_requested:
                self.get_logger().warning("[HALT] 요청됨 → ServoJ 스트림 중단")
                try:
                    cobot.MotionHalt()
                except Exception:
                    pass
                break

            now = time.perf_counter()
            telapsed = now - t0

            # 경로 종료 + 홀드 t2를 지나면 루프 종료
            if telapsed > (ts[-1] + t2):
                break

            # 현재 구간 인덱스 계산
            if telapsed <= ts[0]:
                seg = 0
                s_seg = 0.0
            elif telapsed >= ts[-1]:
                seg = max(0, total_waypoints - 2)
                s_seg = 1.0
            else:
                seg = int(telapsed // wp_dt_scaled)
                seg = min(seg, total_waypoints - 2)
                t0_seg = ts[seg]
                t1_seg = ts[seg + 1]
                denom = max(t1_seg - t0_seg, 1e-9)
                s_seg = (telapsed - t0_seg) / denom
                if s_seg < 0.0: s_seg = 0.0
                if s_seg > 1.0: s_seg = 1.0

            # 선형 보간 목표
            q0 = joint_wps[seg]['q']
            q1 = joint_wps[seg + 1]['q'] if seg + 1 < total_waypoints else joint_wps[-1]['q']
            q_cmd_yaml = [_lerp(q0[i], q1[i], s_seg) for i in range(6)]

            # 내부 블렌드(초기 변화 완화)
            s_blend = _quintic_blend(telapsed / max(internal_blend, 1e-6))
            q_cmd = [_lerp(q_ref_prev[i], q_cmd_yaml[i], s_blend) for i in range(6)]

            # 외부 LPF 적용(필요 시)
            if 0.0 < filter_alpha < 1.0:
                q_ref = [(1.0 - filter_alpha) * q_ref_prev[i] + filter_alpha * q_cmd[i] for i in range(6)]
            else:
                q_ref = q_cmd

            # ServoJ 전송(컨트롤러 파라미터는 원래 값 유지)
            try:
                j = cobot.Joint(*q_ref)
                cobot.ServoJ(j.j0, j.j1, j.j2, j.j3, j.j4, j.j5, t1, t2, gain, servo_alpha)
            except Exception as e:
                self.get_logger().error(f"ServoJ 전송 오류: {e}")
                try:
                    cobot.MotionHalt()
                except Exception:
                    pass
                break

            if seg != last_seg:
                last_seg = seg
                # 진행 퍼블리시
                try:
                    self.progress_pub.publish(Int32(data=min(seg + 1, total_waypoints)))
                except Exception:
                    pass
                # ← 추가: 세그 넘어갈 때 한 줄 로그
                if self._log_each_wp:
                    nxt = min(seg + 1, total_waypoints - 1)
                    self.get_logger().info(
                        f"[ServoJ] wp {nxt}/{total_waypoints-1} target={joint_wps[nxt]['q'][:6]}"
                    )

            k += 1
            t_next = t0 + k * Ts
            _sleep_until(t_next)

            q_ref_prev = q_ref

        end_perf = time.perf_counter()
        end_wall = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        elapsed = end_perf - start_perf
        self.get_logger().info("✅ Servo-streaming(보간) 모션 실행 완료! 총 " + str(total_waypoints) + "개 웨이포인트 기준")
        self.get_logger().info(f"⏱ _run_joint_servoJ 종료: {end_wall} | 총 경과 {elapsed:.3f}s")
        self._publish_duration(elapsed)

    def _run_joint_servoJ_hold(self, joint_wps: List[Dict[str, Any]], params: Dict[str, Any]):
        """Unity 브리지와 동일한 step-hold 방식: 각 wp를 wp_dt 동안 그대로 유지하며 ServoJ 반복 송신."""
        total_waypoints = len(joint_wps)
        if total_waypoints < 1:
            self.get_logger().error("❌ 조인트 웨이포인트가 없습니다.")
            return

        # 파라미터 병합(파일 → 노드 → GUI 오버라이드)
        t1 = float(params.get('servo_t1', self.get_parameter('servo_t1').value))
        t2 = float(params.get('servo_t2', self.get_parameter('servo_t2').value))
        gain = float(params.get('servo_gain', self.get_parameter('servo_gain').value))
        servo_alpha = float(params.get('servo_alpha', self.get_parameter('servo_alpha').value))
        if getattr(self, '_override_enabled', True):
            ov = getattr(self, 'servo_overrides', {}) or {}
            if 't1' in ov: t1 = float(ov['t1'])
            if 't2' in ov: t2 = float(ov['t2'])
            if 'gain' in ov: gain = float(ov['gain'])
            if 'alpha' in ov: servo_alpha = float(ov['alpha'])
        Ts = float(params.get('servo_Ts', 0.008))
        Ts = max(0.002, min(Ts, 0.05))
        wp_dt = float(params.get('wp_dt', 0.1))
        time_scale = float(params.get('time_scale', 1.0))
        wp_dt_scaled = max(1e-4, wp_dt * max(0.1, min(time_scale, 10.0)))
        reps = max(1, int(round(wp_dt_scaled / Ts)))

        def _sleep_until(target: float):
            now = time.perf_counter()
            remain = target - now
            if remain > 0.0015:
                time.sleep(remain - 0.0007)
            while time.perf_counter() < target:
                pass

        start_wall = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        start_perf = time.perf_counter()
        self.get_logger().info(f"⏱ _run_joint_servoJ_hold 시작: {start_wall}")
        self.get_logger().info(f"▶ Ts={Ts:.4f}s, wp_dt={wp_dt:.3f}s, reps={reps}, t1={t1}, t2={t2}, gain={gain}, alpha={servo_alpha}")

        k = 0
        t0 = time.perf_counter()
        for idx, m in enumerate(joint_wps, start=1):
            if self._cancel_requested:
                self.get_logger().warning("[HALT] 요청됨 → ServoJ 스트림 중단")
                try:
                    cobot.MotionHalt()
                except Exception:
                    pass
                break
            q = m['q']
            # 현재 wp를 Ts 주기로 reps 번 송신(hold)
            for r in range(reps):
                if self._cancel_requested:
                    break
                try:
                    j = cobot.Joint(*q)
                    cobot.ServoJ(j.j0, j.j1, j.j2, j.j3, j.j4, j.j5, t1, t2, gain, servo_alpha)
                except Exception as e:
                    self.get_logger().error(f"ServoJ 전송 오류: {e}")
                    try:
                        cobot.MotionHalt()
                    except Exception:
                        pass
                    return
                k += 1
                t_next = t0 + k * Ts
                _sleep_until(t_next)
            # 진행 퍼블리시/로그
            try:
                self.progress_pub.publish(Int32(data=idx))
            except Exception:
                pass
            if self._log_each_wp:
                self.get_logger().info(f"[ServoJ HOLD] wp {idx}/{total_waypoints} q={q[:6]}")

        end_perf = time.perf_counter()
        elapsed = end_perf - start_perf
        self.get_logger().info("✅ Servo-streaming(hold) 모션 실행 완료!")
        self._publish_duration(elapsed)
    # ---------- Emergency Stop / Cancel ----------
    def request_cancel(self):
        """요청 중단: 현재 실행 중 스트리밍 루프를 빠르게 종료하고 로봇을 정지."""
        self._cancel_requested = True
        try:
            cobot.MotionHalt()
        except Exception:
            pass
        self.get_logger().warning("[HALT] cancel 요청 전송 (MotionHalt)")

    def _run_cartesian_path(self, cart_wps: List[Dict[str, Any]], opt, rtype):
        """Simple cartesian runner using MovePB (unchanged)."""
        # Clear
        if hasattr(cobot, 'MovePB_Clear'):
            cobot.MovePB_Clear()

        # Add
        for m in cart_wps:
            x, y, z, rx, ry, rz = m['pose']
            p = cobot.Point(_as_float(x), _as_float(y), _as_float(z),
                            _as_float(rx), _as_float(ry), _as_float(rz))
            cobot.MovePB_Add(p, float(m['speed']), opt, float(m['blend']))
            self.get_logger().info(f"[📥] 카르테시안 추가: {m['pose']}")

        # Run
        try:
            cobot.MovePB_Run(100.0, rtype)
        except TypeError:
            cobot.MovePB_Run(100.0)

        self.get_logger().info("✅ 카르테시안(IK) 모션 실행 완료!")

def main():
    rclpy.init()
    node = MotionExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

