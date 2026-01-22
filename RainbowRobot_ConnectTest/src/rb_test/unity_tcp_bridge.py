#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity → TCP(JSON/YAML lines) → Laptop → Robot(COBOT SDK)
- ROS 불필요. Unity가 0.1s 주기로 한 줄(JSON 또는 YAML)씩 전송(끝에 '\n')
- 수신 형식:
  1) joints 직접 전달
     {"mode":"joint","joints_deg":[10,20,30,40,50,60]}
     {"mode":"joint","joints_rad":[0.1,0.2,0.3,0.4,0.5,0.6]}
  2) jointDataList 전달(요청 사양: link1..6에서 특정 축+W 사용)
     {"jointDataList":[{"name":"link1","rotation":{"y":0.0008,"w":0.9999}}, ... ]}
     - 각 관절 각도 θ = 2*atan2(comp, w) [deg], comp 축은 joint_axis_map로 지정
  3) 설정 변경
     {"cfg":{"robot_ip":"192.168.1.13","mode":"joint","joint_axis_map":["y","x","x","x","y","x"]}}
- 송신:
  - mode=joint → 0.008s(step-hold) ServoJ 지속 송신, 0.1s에 들어오는 최신 joints만 갱신
  - mode=tcp   → (옵션) 카르테시안 1점 MovePB 실행
"""

import argparse
import asyncio
import json
import math
import os
import socket
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

try:
    import yaml  # YAML 라인도 허용
except Exception:
    yaml = None

from rb_test import cobot


def quat_axis_angle_deg(comp: float, w: float) -> float:
    return math.degrees(2.0 * math.atan2(float(comp), float(w if w != 0.0 else 1.0)))


def joints_from_jointDataList(doc: Dict, axis_per_link: List[str], decimals: int = 2) -> Optional[List[float]]:
    lst = doc.get("jointDataList")
    if not isinstance(lst, list) or len(lst) < 6:
        return None
    links: Dict[str, Dict] = {}
    for it in lst:
        name = str(it.get("name", "")).lower()
        if name.startswith("link"):
            links[name] = it
    out: List[float] = []
    for i in range(6):
        key = f"link{i+1}"
        it = links.get(key) if key in links else (lst[i] if i < len(lst) else None)
        if not it:
            return None
        ax = axis_per_link[i].lower()
        rot_q = it.get("rotation")
        # 신형: targetValues / 구형: localEulerAngles
        rot_e = it.get("targetValues") or it.get("localEulerAngles")
        if isinstance(rot_q, dict) and ("w" in rot_q):
            comp = float(rot_q.get(ax, 0.0))
            w = float(rot_q.get("w", 1.0))
            ang = quat_axis_angle_deg(comp, w)              # [deg]
        elif isinstance(rot_e, dict):
            ang = float(rot_e.get(ax, 0.0))                 # [deg]
        else:
            return None
        out.append(round(ang, decimals))
    if len(out) >= 5:
        out[4] = round(-out[4], decimals)
    return out


def map_axes_xyz(src_xyz: Tuple[float, float, float], axis_map: Tuple[str, str, str]) -> Tuple[float, float, float]:
    sx, sy, sz = src_xyz
    d = {'x': sx, 'y': sy, 'z': sz}
    out = []
    for m in axis_map:
        m = m.strip().lower()
        sgn = -1.0 if m.startswith('-') else 1.0
        key = m.lstrip('-+')
        out.append(sgn * d[key])
    return (out[0], out[1], out[2])




class UnityTcpBridge:
    def __init__(self, 
                 robot_ip: str = "192.168.1.13",
                 listen_host: str = "0.0.0.0",
                 listen_port: int = 10001,
                 allowed_client_ip: Optional[str] = None,
                 step_dt: float = 0.1,
                 servo_Ts: float = 0.008,
                 filter_alpha: float = 1.0,
                 joint_axis_map: List[str] = None,
                 joint_round_decimals: int = 2,
                 stale_timeout: float = 0.35):
        # 라이브 모드 기본 송신 간격 = servo_Ts
        try:
            cobot.set_send_min_interval(servo_Ts)
        except Exception:
            pass
        self.robot_ip = robot_ip
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.allowed_client_ip = allowed_client_ip

        self.step_dt = step_dt
        self.servo_Ts = servo_Ts
        self.filter_alpha = max(0.0, min(1.0, float(filter_alpha)))
        self.joint_axis_map = joint_axis_map or ['y', 'x', 'x', 'x', 'y', 'x']
        self.joint_round_decimals = int(joint_round_decimals)
        self.stale_timeout = float(stale_timeout)

        self.mode = "joint"  # 'joint' | 'cartesian'
        self.current_joint_deg: Optional[List[float]] = None
        self.q_ref_prev: Optional[List[float]] = None
        self.last_update_ts: float = 0.0

        # ServoJ 파라미터
        self.t1, self.t2, self.gain, self.alpha = 0.05, 0.05, 0.05, 0.003

        # ==== moved from set_servo_overrides: runtime/record/play init ====
        self._stop_evt = threading.Event()
        self.forwarding_enabled = True
        # recording
        self._rec_lock = threading.Lock()
        self._rec_on = False
        self._rec_start_ts = 0.0
        self._rec_samples: List[Dict] = []
        # playback
        self._play_stop_evt = threading.Event()
        self._play_thread: Optional[threading.Thread] = None
        # single-step guard
        self._step_lock = threading.Lock()
        # robot + servoj loop
        self._connect_robot(self.robot_ip)
        self._servo_thread = threading.Thread(target=self._servoj_loop, daemon=True)
        self._servo_thread.start()
        print(f"[bridge] up listen={self.listen_host}:{self.listen_port} robot_ip={self.robot_ip} "
              f"mode={self.mode} Ts={self.servo_Ts} joint_axis_map={self.joint_axis_map}")
        # ================================================================

        # 재생 종료 안정화 옵션
        self.tail_hold_s: float = 0.6
        self.stop_after_playback: bool = True
        # 재생 상태
        self._play_stop_evt = threading.Event()
        self._play_thread: Optional[threading.Thread] = None

    def set_servo_overrides(self, t1=None, t2=None, gain=None, alpha=None):
        """외부(UI/Runner)에서 Unity 브리지의 ServoJ 파라미터를 동기화하기 위한 API."""
        if t1 is not None:
            self.t1 = float(t1)
        if t2 is not None:
            self.t2 = float(t2)
        if gain is not None:
            self.gain = float(gain)
        if alpha is not None:
            self.alpha = float(alpha)
        print(f"[bridge] ServoJ params set t1={self.t1}, t2={self.t2}, gain={self.gain}, alpha={self.alpha}")

    def is_playing(self) -> bool:
        t = getattr(self, "_play_thread", None)
        return bool(t and t.is_alive())

    def set_tail_hold_s(self, sec: float):
        try:
            self.tail_hold_s = max(0.0, float(sec))
            print(f"[bridge] tail_hold_s={self.tail_hold_s}")
        except Exception:
            pass

    def set_stop_after_playback(self, flag: bool):
        self.stop_after_playback = bool(flag)
        print(f"[bridge] stop_after_playback={self.stop_after_playback}")

    def set_filter_alpha(self, a: float):
        try:
            self.filter_alpha = float(a)
            print(f"[bridge] filter_alpha set to {self.filter_alpha}")
        except Exception as e:
            print(f"[bridge] set_filter_alpha failed: {e}")

    def _connect_robot(self, ip: str):
        try:
            cobot.ConnectToCB(ip)
            print(f"[bridge] robot connected {ip}")
        except Exception as e:
            print(f"[bridge] robot connect failed: {e}")

    async def start(self):
        server = await asyncio.start_server(self._on_client, self.listen_host, self.listen_port)
        addrs = ", ".join(str(s.getsockname()) for s in server.sockets)
        print(f"[bridge] listening {addrs}")
        async with server:
            await server.serve_forever()

    async def _on_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        peer = writer.get_extra_info("peername")
        peer_ip = peer[0] if isinstance(peer, tuple) else str(peer)
        print(f"[bridge] client connect {peer}")
        if self.allowed_client_ip and peer_ip != self.allowed_client_ip:
            print(f"[bridge] deny {peer_ip} (allowed={self.allowed_client_ip})")
            writer.close()
            try:
                await writer.wait_closed()
            finally:
                return
        try:
            buf = ""            # ← 멀티라인 누적 버퍼
            last_feed = time.perf_counter()
            MAX_BUF = 128 * 1024
            while not reader.at_eof():
                line = await reader.readline()
                if not line:
                    break
                text_line = line.decode(errors="ignore")
                print(f"[bridge] << {text_line.rstrip()[:200]}")  # 수신 라인 프리뷰

                # 새 프레임 시작 신호 감지: 'id:' 또는 YAML 문서 구분자 '---'
                stripped = text_line.lstrip()
                new_frame_starts = stripped.startswith("id:") or stripped.startswith("---")
                if new_frame_starts and buf.strip():
                    # 직전 버퍼를 한 문서로 처리
                    ready, ok, resp = self._try_parse_message(buf)
                    if ready:
                        try:
                            writer.write((json.dumps({"ok": ok, **(resp or {})}) + "\n").encode())
                            await writer.drain()
                        except Exception:
                            pass
                    buf = ""  # 새 문서 시작을 위해 버퍼 리셋

                # 빈 줄이 오면 '문서 종료'로 간주
                is_blank = (text_line.strip() == "")
                buf += text_line
                if len(buf) > MAX_BUF:
                    buf = ""   # 과도 버퍼 방어
                    try:
                        writer.write((json.dumps({"ok": False, "err": "too_large"}) + "\n").encode()); await writer.drain()
                    except Exception:
                        pass
                    continue

                # 버퍼를 시도 파싱(완성 문서인지 판단)
                ready, ok, resp = self._try_parse_message(buf)
                if ready:
                    try:
                        writer.write((json.dumps({"ok": ok, **(resp or {})}) + "\n").encode())
                        await writer.drain()
                    except Exception:
                        pass
                    buf = ""     # 처리 완료 → 버퍼 초기화
                    last_feed = time.perf_counter()
                    continue

                # 아직 미완성인데 빈 줄로 종료 신호가 오면 에러 응답 후 버퍼 초기화
                if is_blank:
                    try:
                        writer.write((json.dumps({"ok": False, "err": "invalid"}) + "\n").encode())
                        await writer.drain()
                    except Exception:
                        pass
                    buf = ""
                    last_feed = time.perf_counter()
                    continue

            # 연결 종료 시 버퍼 잔여물이 있으면 마지막으로 시도
            if buf.strip():
                ready, ok, resp = self._try_parse_message(buf)
                if ready:
                    try:
                        writer.write((json.dumps({"ok": ok, **(resp or {})}) + "\n").encode())
                        await writer.drain()
                    except Exception:
                        pass
        except Exception as e:
            print(f"[bridge] client error {peer}: {e}")
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass
            print(f"[bridge] client closed {peer}")

    def _parse_jointdatalist_yaml_fallback(self, text: str) -> Optional[List[float]]:
        """
        PyYAML이 없거나 YAML 파싱이 실패할 때를 위한 폴백 파서.
        jointDataList의 localEulerAngles에서 link1..6의 x/y/z를 라인 스캔으로 추출.
        모든 링크가 준비되었을 때만 완료로 간주.
        """
        lines = text.splitlines()
        cur = None
        in_angles = False
        store: Dict[str, Dict[str, float]] = {}
        for line in lines:
            s = line.strip()
            if not s:
                continue
            if s.startswith('- name:'):
                # 새 링크 시작
                try:
                    name = s.split(':', 1)[1].strip().lower()
                except Exception:
                    name = ""
                cur = name if name.startswith("link") else None
                in_angles = False
                continue
            if s.startswith('targetValues:') or s.startswith('localEulerAngles:'):
                in_angles = True
                if cur and cur not in store:
                    store[cur] = {}
                continue
            if in_angles and cur and (s.startswith('x:') or s.startswith('y:') or s.startswith('z:')):
                try:
                    k, v = s.split(':', 1)
                    store[cur][k.strip()] = float(v.strip())
                except Exception:
                    pass

        # 모든 링크가 있어야 완료로 인정
        needed = [f"link{i+1}" for i in range(6)]
        if not all(k in store for k in needed):
            return None

        out: List[float] = []
        for i in range(6):
            key = f"link{i+1}"
            rot_e = store.get(key, {})
            ax = self.joint_axis_map[i].lower()
            if ax not in rot_e:
                return None
            ang = float(rot_e.get(ax, 0.0))
            out.append(round(ang, self.joint_round_decimals))
        # 5축만 부호 반전
        if len(out) >= 5:
            out[4] = round(-out[4], self.joint_round_decimals)
        return out

    def _jdl_complete(self, doc: Dict) -> bool:
        lst = doc.get("jointDataList")
        if not isinstance(lst, list) or len(lst) < 6:
            return False
        links = {}
        for it in lst:
            n = str(it.get("name", "")).lower()
            if n.startswith("link"):
                links[n] = it
        for i in range(6):
            key = f"link{i+1}"
            it = links.get(key)
            if not isinstance(it, dict):
                return False
            ax = self.joint_axis_map[i].lstrip("+-").lower()
            rot_q = it.get("rotation")
            rot_e = it.get("targetValues") or it.get("localEulerAngles")
            if isinstance(rot_q, dict):
                if ("w" not in rot_q) or (ax not in rot_q):
                    return False
            elif isinstance(rot_e, dict):
                if ax not in rot_e:
                    return False
            else:
                return False
        return True

    def _try_parse_message(self, text: str):
        """
        멀티라인 버퍼를 파싱.
        반환: (ready, ok, resp)
        """
        doc = None
        # JSON 우선
        try:
            doc = json.loads(text)
        except Exception:
            # YAML 시도
            if yaml:
                try:
                    doc = yaml.safe_load(text)
                except Exception:
                    doc = None

        # 폴백: 라인 스캔
        if doc is None:
            q_fb = self._parse_jointdatalist_yaml_fallback(text)
            if q_fb:
                self.current_joint_deg = q_fb
                self.last_update_ts = time.perf_counter()
                self.mode = "joint"
                self._on_new_sample(q_fb)
                print(f"[bridge] RX jointDataList(fallback) → joints_deg {q_fb}")
                return (True, True, {"mode": "joint", "accepted": True})
            # 아직 미완성일 수 있음
            return (False, False, None)

        if not isinstance(doc, dict):
            return (False, False, None)

        # 미완성은 기다림
        if isinstance(doc.get("joints_deg"), list) and len(doc["joints_deg"]) < 6:
            return (False, False, None)
        if isinstance(doc.get("joints_rad"), list) and len(doc["joints_rad"]) < 6:
            return (False, False, None)
        if "jointDataList" in doc and not self._jdl_complete(doc):
            return (False, False, None)

        # cfg 즉시 처리
        if "cfg" in doc and isinstance(doc["cfg"], dict):
            ok, resp = self._process_cfg(doc["cfg"])
            return (True, ok, resp)

        # 완성 본문 처리
        if isinstance(doc.get("joints_deg"), list) or isinstance(doc.get("joints_rad"), list) or isinstance(doc.get("jointDataList"), list):
            ok, resp = self._process_payload(doc)
            return (True, ok, resp)

        return (False, False, None)

    def _process_cfg(self, cfg: Dict):
        if "robot_ip" in cfg:
            ip = str(cfg["robot_ip"])
            if ip and ip != self.robot_ip:
                self.robot_ip = ip
                self._connect_robot(ip)
        if "mode" in cfg:
            self.mode = "joint" if str(cfg["mode"]).lower().startswith("joint") else "cartesian"
        if "joint_axis_map" in cfg:
            amap = cfg["joint_axis_map"]
            if isinstance(amap, list) and len(amap) == 6:
                self.joint_axis_map = [str(v) for v in amap]
        if "servo_Ts" in cfg:
            try:
                self.servo_Ts = float(cfg["servo_Ts"])
            except Exception:
                pass
        if "filter_alpha" in cfg:
            try:
                self.filter_alpha = max(0.0, min(1.0, float(cfg["filter_alpha"])))
            except Exception:
                pass
        return True, {"cfg_applied": True, "mode": self.mode}

    def _process_payload(self, doc: Dict):
        # joints 직접
        if isinstance(doc.get("joints_deg"), list) or isinstance(doc.get("joints_rad"), list):
            q = None
            try:
                if isinstance(doc.get("joints_deg"), list) and len(doc["joints_deg"]) >= 6:
                    q = [round(float(v), self.joint_round_decimals) for v in doc["joints_deg"][:6]]
                elif isinstance(doc.get("joints_rad"), list) and len(doc["joints_rad"]) >= 6:
                    q = [round(math.degrees(float(v)), self.joint_round_decimals) for v in doc["joints_rad"][:6]]
            except Exception:
                q = None
            if q:
                self.current_joint_deg = q
                self.last_update_ts = time.perf_counter()
                self.mode = "joint"
                self._on_new_sample(q)
                print(f"[bridge] RX joints_deg {q}")
                return True, {"mode": "joint", "accepted": True}
            return False, {"err": "bad_joints"}

        # jointDataList → joints_deg
        if isinstance(doc.get("jointDataList"), list):
            q = joints_from_jointDataList(doc, self.joint_axis_map, self.joint_round_decimals)
            if q:
                self.current_joint_deg = q
                self.last_update_ts = time.perf_counter()
                self.mode = "joint"
                self._on_new_sample(q)
                print(f"[bridge] RX jointDataList → joints_deg {q}")
                return True, {"mode": "joint", "accepted": True}
            return False, {"err": "bad_jointDataList"}

        return False, {"err": "unknown"}

    # ---- 제어 API ----
    def set_forwarding(self, on: bool):
        self.forwarding_enabled = bool(on)
        print(f"[bridge] forwarding={'ON' if self.forwarding_enabled else 'OFF'}")

    def get_latest_joint(self) -> Optional[List[float]]:
        return list(self.current_joint_deg) if self.current_joint_deg else None

    def get_recording_count(self) -> int:
        with self._rec_lock:
            return len(self._rec_samples)

    def start_record(self):
        with self._rec_lock:
            self._rec_on = True
            self._rec_start_ts = time.perf_counter()
            self._rec_samples.clear()
        print("[bridge] recording START")

    def stop_record_and_save(self, base_dir: str, wp_dt: float = 0.1) -> Optional[str]:
        # 날짜별 디렉터리로 저장
        with self._rec_lock:
            data = list(self._rec_samples)
            self._rec_on = False
        if not data:
            print("[bridge] recording STOP (no data)")
            return None
        day = datetime.now().strftime("%Y%m%d")
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        day_dir = os.path.join(base_dir, day)
        os.makedirs(day_dir, exist_ok=True)
        name = f"unity_{ts}"
        path = os.path.join(day_dir, f"{name}.yaml")
        # YAML 포맷(motion 파일 스타일)
        doc = {
            "motions": [
                {
                    "name": name,
                    "motion_type": "joint_path",
                    "execution_mode": "j_add",
                    "coordinate_frame": "robot_base_frame",
                    "parameters": {
                        "wp_dt": float(wp_dt),
                        # ↓ 추가: 라이브 포워딩 시 사용한 파라미터를 함께 저장
                        "play_mode": "hold",                  # 라이브와 동일(스텝-홀드)
                        "servo_t1": float(self.t1),
                        "servo_t2": float(self.t2),
                        "servo_gain": float(self.gain),
                        "servo_alpha": float(self.alpha),
                        "filter_alpha": float(self.filter_alpha),
                        "joint_round_decimals": int(self.joint_round_decimals),
                        "joint_axis_map": list(self.joint_axis_map),
                        "flip_j5": True,                      # 5축 부호 반전 적용 사실 표기
                        "source": "unity_tcp_bridge",
                    },
                    "waypoints": [
                        {"id": i, "type": "joint", "joints": s["joints"]}
                        for i, s in enumerate(data)
                    ],
                }
            ]
        }
        try:
            if yaml:
                with open(path, "w", encoding="utf-8") as f:
                    yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
            else:
                # YAML 없는 경우 JSON으로 저장
                path = path.replace(".yaml", ".json")
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(doc, f, ensure_ascii=False, indent=2)
            print(f"[bridge] recording SAVE → {path} (samples={len(data)})")
            return path
        except Exception as e:
            print(f"[bridge] save failed: {e}")
            return None

    def _on_new_sample(self, joints_deg: List[float]):
        # 수신 샘플 기록
        with self._rec_lock:
            if self._rec_on:
                t = time.perf_counter() - self._rec_start_ts
                self._rec_samples.append({"t": float(t), "joints": list(joints_deg)})

    def set_current_joint(self, joints_deg: List[float]):
        # 수동/재생 전송 시 사용
        self.current_joint_deg = [float(round(v, self.joint_round_decimals)) for v in joints_deg]
        self.last_update_ts = time.perf_counter()

    def playback_start(self, seq: List[List[float]], wp_dt: float = 0.1):
        self.playback_stop()
        self._play_stop_evt.clear()
        def _run():
            print(f"[bridge] playback START (N={len(seq)}, dt={wp_dt})")
            self.set_forwarding(True)
            last_q = None
            for q in seq:
                if self._play_stop_evt.is_set():
                    break
                last_q = q
                self.set_current_joint(q)
                time.sleep(max(0.0, float(wp_dt)))
            # 재생 종료 → 마지막 점을 짧게 유지해 잔류 억제
            if last_q is not None and not self._play_stop_evt.is_set():
                prev_f = float(self.filter_alpha)
                try:
                    # 잔류 제거 위해 일시적으로 필터 제거
                    self.set_filter_alpha(1.0)
                except Exception:
                    pass
                Ts = max(0.002, float(self.servo_Ts))
                n = max(1, int(round(float(self.tail_hold_s) / Ts)))
                for _ in range(n):
                    if self._play_stop_evt.is_set():
                        break
                    self.set_current_joint(last_q)
                    time.sleep(Ts)
                try:
                    self.set_filter_alpha(prev_f)
                except Exception:
                    pass
            # 옵션: 재생 후 포워딩 종료
            if self.stop_after_playback and not self._play_stop_evt.is_set():
                try:
                    self.set_forwarding(False)
                except Exception:
                    pass
            print("[bridge] playback END")
        self._play_thread = threading.Thread(target=_run, daemon=True)
        self._play_thread.start()

    def playback_stop(self):
        self._play_stop_evt.set()
        th = self._play_thread
        if th and th.is_alive():
            th.join(timeout=0.5)
        self._play_thread = None

    def send_step_once(self, joints_deg: List[float], duration_s: float = 0.2):
        """라이브 포워딩과 무관하게 ServoJ를 duration_s 동안 반복 송신(단일 스텝 펄스)."""
        q = [float(round(v, self.joint_round_decimals)) for v in joints_deg[:6]]
        Ts = max(0.002, float(self.servo_Ts))
        n = max(1, int(duration_s / Ts))
        t1, t2, gain, alpha = self.t1, self.t2, self.gain, self.alpha

        def _pulse():
            with self._step_lock:
                for _ in range(n):
                    try:
                        j = cobot.Joint(*q)
                        cobot.ServoJ(j.j0, j.j1, j.j2, j.j3, j.j4, j.j5, t1, t2, gain, alpha)
                    except Exception as e:
                        print(f"[bridge] step ServoJ failed: {e}")
                        break
                    time.sleep(Ts)
        threading.Thread(target=_pulse, daemon=True).start()

    def _process_line(self, text: str):
        # JSON 우선, 실패 시 YAML 허용
        try:
            doc = json.loads(text)
        except Exception:
            if yaml:
                try:
                    doc = yaml.safe_load(text)
                except Exception:
                    doc = None
            else:
                doc = None
        if not isinstance(doc, dict):
            return False, {"err": "invalid"}

        # cfg 적용
        if "cfg" in doc and isinstance(doc["cfg"], dict):
            cfg = doc["cfg"]
            if "robot_ip" in cfg:
                ip = str(cfg["robot_ip"])
                if ip and ip != self.robot_ip:
                    self.robot_ip = ip
                    self._connect_robot(ip)
            if "mode" in cfg:
                self.mode = "joint" if str(cfg["mode"]).lower().startswith("joint") else "cartesian"
            if "joint_axis_map" in cfg:
                amap = cfg["joint_axis_map"]
                if isinstance(amap, list) and len(amap) == 6:
                    self.joint_axis_map = [str(v) for v in amap]
            if "servo_Ts" in cfg:
                try:
                    self.servo_Ts = float(cfg["servo_Ts"])
                except Exception:
                    pass
            if "filter_alpha" in cfg:
                try:
                    self.filter_alpha = max(0.0, min(1.0, float(cfg["filter_alpha"])))
                except Exception:
                    pass
            return True, {"cfg_applied": True, "mode": self.mode}

        mode = str(doc.get("mode", self.mode)).lower()

        # joints 직접
        if isinstance(doc.get("joints_deg"), list) or isinstance(doc.get("joints_rad"), list):
            q = None
            try:
                if isinstance(doc.get("joints_deg"), list) and len(doc["joints_deg"]) >= 6:
                    q = [round(float(v), self.joint_round_decimals) for v in doc["joints_deg"][:6]]
                elif isinstance(doc.get("joints_rad"), list) and len(doc["joints_rad"]) >= 6:
                    q = [round(math.degrees(float(v)), self.joint_round_decimals) for v in doc["joints_rad"][:6]]
            except Exception:
                q = None
            if q:
                self.current_joint_deg = q
                self.last_update_ts = time.perf_counter()
                self.mode = "joint"
                self._on_new_sample(q)  # ← 추가: 녹화 버퍼에 반영
                print(f"[bridge] RX joints_deg {q}")
                return True, {"mode": "joint", "accepted": True}
            return False, {"err": "bad_joints"}

        # jointDataList → joints_deg
        if isinstance(doc.get("jointDataList"), list):
            q = joints_from_jointDataList(doc, self.joint_axis_map, self.joint_round_decimals)
            if q:
                self.current_joint_deg = q
                self.last_update_ts = time.perf_counter()
                self.mode = "joint"
                self._on_new_sample(q)  # ← 추가
                print(f"[bridge] RX jointDataList → joints_deg {q}")
                return True, {"mode": "joint", "accepted": True}
            return False, {"err": "bad_jointDataList"}

        # (옵션) 카르테시안 포즈 처리 필요 시 여기에 추가
        return False, {"err": "unknown"}

    def _servoj_loop(self):
        Ts = float(self.servo_Ts)
        t0 = time.perf_counter()
        k = 0
        while not self._stop_evt.is_set():
            now = time.perf_counter()
            if self.mode == "joint" and self.current_joint_deg is not None and self.forwarding_enabled:
                # 최신값이 너무 오래되면 보류
                if self.last_update_ts > 0 and (now - self.last_update_ts) <= self.stale_timeout:
                    q = self.current_joint_deg
                    if q is not None:
                        if self.q_ref_prev is None:
                            self.q_ref_prev = list(q)
                        filt = float(self.filter_alpha)
                        q_ref = [(1.0 - filt) * self.q_ref_prev[i] + filt * q[i] for i in range(6)] if 0.0 < filt < 1.0 else list(q)
                        try:
                            j = cobot.Joint(*q_ref)
                            cobot.ServoJ(
                                j.j0, j.j1, j.j2, j.j3, j.j4, j.j5,
                                float(self.t1), float(self.t2), float(self.gain), float(self.alpha)
                            )
                        except Exception:
                            pass
                        self.q_ref_prev = q_ref
            # 타이밍 정렬
            t_next = t0 + k * Ts
            remain = t_next - time.perf_counter()
            if remain > 0:
                time.sleep(min(0.002, remain))
            k += 1

    def stop(self):
        self._stop_evt.set()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=10001)
    ap.add_argument("--robot-ip", default="192.168.1.13")
    ap.add_argument("--allow", dest="allow", default=None, help="허용할 Unity IP(예: 192.168.56.1)")
    args = ap.parse_args()

    bridge = UnityTcpBridge(
        robot_ip=args.robot_ip,
        listen_host=args.listen,
        listen_port=args.port,
        allowed_client_ip=args.allow,
        joint_axis_map=['y','x','x','x','y','x'],  # 요청 축 매핑
        joint_round_decimals=2,
        servo_Ts=0.008,
        filter_alpha=1.0,
    )
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()