#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity TCP Recorder/Player GUI
- 0.1s 수신 → 즉시 송신(ServoJ) 토글
- 저장 시작/정지(날짜별 디렉터리 ~/Downloads/smart_ws_v2/unity_motions/YYYMMDD)
- 파일/버퍼 재생(0.1s), 단일 스텝 전송
실행 예:
  python3 src/rb_test/rb_test/unity_tcp_rec_gui.py --listen 192.168.1.100 --port 10001 --robot-ip 192.168.1.13 --allow 192.168.56.1
"""
import argparse
import asyncio
import json
import os
from pathlib import Path
import threading
import time
from typing import List, Optional

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFileDialog, QCheckBox, QDoubleSpinBox
)
from PyQt5.QtCore import QTimer, Qt
from datetime import datetime  # ← 추가

try:
    import yaml
except Exception:
    yaml = None

# joints_from_jointDataList 재사용(없으면 fallback 정의)
try:
    from rb_test.unity_tcp_bridge import UnityTcpBridge, joints_from_jointDataList
except Exception:
    from rb_test.unity_tcp_bridge import UnityTcpBridge
    def joints_from_jointDataList(doc, axis_per_link: List[str], decimals: int = 2):
        lst = doc.get("jointDataList")
        if not isinstance(lst, list) or len(lst) < 6:
            return None
        links = {}
        for it in lst:
            name = str(it.get("name", "")).lower()
            if name.startswith("link"):
                links[name] = it
        out = []
        import math
        for i in range(6):
            key = f"link{i+1}"
            it = links.get(key) if key in links else (lst[i] if i < len(lst) else None)
            if not it:
                return None
            ax = axis_per_link[i].lower()
            rot_q = it.get("rotation")
            rot_e = it.get("targetValues") or it.get("localEulerAngles")
            if isinstance(rot_q, dict) and ("w" in rot_q):
                comp = float(rot_q.get(ax, 0.0))
                w = float(rot_q.get("w", 1.0))
                ang = math.degrees(2.0 * math.atan2(comp, w if w != 0.0 else 1.0))
            elif isinstance(rot_e, dict):
                ang = float(rot_e.get(ax, 0.0))
            else:
                return None
            out.append(round(ang, decimals))
        # 5축만 부호 반전
        if len(out) >= 5:
            out[4] = round(-out[4], decimals)
        return out

def _guess_workspace_root() -> str:
    # 1) SMART_WS_DIR 우선
    env = os.environ.get("SMART_WS_DIR")
    if env:
        return str(Path(env).expanduser().resolve())
    # 2) 이 파일: <ws>/src/rb_test/rb_test/unity_tcp_rec_gui.py
    try:
        here = Path(__file__).resolve()
        return str(here.parents[3])  # <ws>
    except Exception:
        return str(Path.cwd().resolve())

BASE_SAVE_DIR = str(Path(_guess_workspace_root()) / "unity_motions")

# jointDataList 프레임들을 파일에서 추출
def extract_joint_seq_from_jointdatalist_file(path: str,
                                              axis_map: List[str],
                                              decimals: int = 2) -> Optional[List[List[float]]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception:
        return None

    frames: List[dict] = []

    # 1) 멀티 도큐먼트 YAML 시도
    if yaml:
        try:
            for doc in yaml.safe_load_all(text):
                if isinstance(doc, dict):
                    if "jointDataList" in doc:
                        frames.append(doc)
                    # 리스트 안에 프레임들이 들어있는 경우
                    elif isinstance(doc.get("frames"), list):
                        for it in doc["frames"]:
                            if isinstance(it, dict) and "jointDataList" in it:
                                frames.append(it)
        except Exception:
            frames = []

    # 2) 단일 YAML/JSON 도큐먼트 시도
    if not frames:
        doc = None
        if yaml:
            try:
                doc = yaml.safe_load(text)
            except Exception:
                doc = None
        if doc is None:
            try:
                doc = json.loads(text)
            except Exception:
                doc = None
        if isinstance(doc, list):
            for it in doc:
                if isinstance(it, dict) and "jointDataList" in it:
                    frames.append(it)
        elif isinstance(doc, dict):
            if "jointDataList" in doc:
                frames.append(doc)
            elif isinstance(doc.get("frames"), list):
                for it in doc["frames"]:
                    if isinstance(it, dict) and "jointDataList" in it:
                        frames.append(it)

    if not frames:
        return None

    seq: List[List[float]] = []
    for fr in frames:
        q = joints_from_jointDataList(fr, axis_map, decimals)
        if q:
            seq.append(q)
    return seq if seq else None


def run_server_in_thread(bridge: UnityTcpBridge):
    def _runner():
        try:
            asyncio.run(bridge.start())
        except Exception as e:
            print(f"[gui] server loop ended: {e}")
    th = threading.Thread(target=_runner, daemon=True)
    th.start()
    return th


def load_joint_seq_from_file(path: str) -> Optional[List[List[float]]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        doc = None
        # YAML 우선
        if yaml:
            try:
                doc = yaml.safe_load(text)
            except Exception:
                doc = None
        if doc is None:
            doc = json.loads(text)
    except Exception:
        return None

    # motions → waypoints
    if isinstance(doc, dict) and isinstance(doc.get("motions"), list):
        mot = doc["motions"][0]
        wps = mot.get("waypoints") or []
        seq = []
        for wp in wps:
            q = wp.get("joints") or wp.get("q")
            if isinstance(q, list) and len(q) >= 6:
                seq.append([float(q[i]) for i in range(6)])
        return seq if seq else None
    # 단순 리스트
    if isinstance(doc, list) and doc and isinstance(doc[0], list):
        return [[float(v) for v in row[:6]] for row in doc]
    return None


class TcpRecorderWindow(QWidget):
    def __init__(self, bridge: UnityTcpBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("Unity TCP Recorder/Player")
        self.resize(680, 260)

        self.seq: Optional[List[List[float]]] = None
        self.seq_idx = 0

        lay = QVBoxLayout(self)

        # 상태 라벨
        self.lbl_j = QLabel("최근 Joints(deg): (없음)")
        self.lbl_cnt = QLabel("기록 샘플: 0")
        self.lbl_file = QLabel("파일: (없음)")
        self.lbl_status = QLabel("상태: 대기")
        for w in (self.lbl_j, self.lbl_cnt, self.lbl_file, self.lbl_status):
            w.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lay.addWidget(w)

        # 상단 컨트롤: 라이브 포워딩 ON/OFF, 재생 dt
        row0 = QHBoxLayout()
        self.chk_forward = QCheckBox("라이브 포워딩(즉시 송신)")
        self.chk_forward.setChecked(True)
        self.chk_forward.toggled.connect(self.on_toggle_forward)
        row0.addWidget(self.chk_forward)

        row0.addWidget(QLabel("재생 간격(s):"))
        self.spin_dt = QDoubleSpinBox()
        self.spin_dt.setDecimals(3)
        self.spin_dt.setRange(0.001, 1.0)
        self.spin_dt.setSingleStep(0.01)
        self.spin_dt.setValue(0.05)
        row0.addWidget(self.spin_dt)
        row0.addStretch(1)
        lay.addLayout(row0)

        # 녹화 버튼
        row1 = QHBoxLayout()
        btn_rec_start = QPushButton("저장 시작")
        btn_rec_stop = QPushButton("저장 정지(파일 저장)")
        btn_rec_start.clicked.connect(self.on_rec_start)
        btn_rec_stop.clicked.connect(self.on_rec_stop)
        row1.addWidget(btn_rec_start)
        row1.addWidget(btn_rec_stop)
        lay.addLayout(row1)

        # 파일 열기/재생/정지/단일스텝
        row2 = QHBoxLayout()
        btn_open = QPushButton("파일 열기(자동 변환/저장 지원)")
        btn_play = QPushButton("재생 시작")
        btn_stop = QPushButton("재생 정지")
        btn_step = QPushButton("단일 스텝 전송")
        btn_reset = QPushButton("처음으로")
        btn_open.clicked.connect(self.on_open_file)
        btn_play.clicked.connect(self.on_play_start)
        btn_stop.clicked.connect(self.on_play_stop)
        btn_step.clicked.connect(self.on_step_once)
        btn_reset.clicked.connect(self.on_reset_index)
        row2.addWidget(btn_open)
        row2.addWidget(btn_play)
        row2.addWidget(btn_stop)
        row2.addWidget(btn_step)
        row2.addWidget(btn_reset)
        lay.addLayout(row2)

        # 주기적 상태 갱신
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def refresh(self):
        q = self.bridge.get_latest_joint()
        if q:
            self.lbl_j.setText("최근 Joints(deg): " + ", ".join(f"{v:.2f}" for v in q))
        else:
            self.lbl_j.setText("최근 Joints(deg): (없음)")
        self.lbl_cnt.setText(f"기록 샘플: {self.bridge.get_recording_count()}")

    def on_toggle_forward(self, on: bool):
        self.bridge.set_forwarding(on)
        self.lbl_status.setText(f"상태: 라이브 포워딩 {'ON' if on else 'OFF'}")

    def on_rec_start(self):
        os.makedirs(BASE_SAVE_DIR, exist_ok=True)
        self.bridge.start_record()
        self.lbl_status.setText("상태: 저장 시작")

    def on_rec_stop(self):
        path = self.bridge.stop_record_and_save(BASE_SAVE_DIR, wp_dt=float(self.spin_dt.value()))
        if path:
            self.lbl_file.setText(f"파일: {path}")
            self.lbl_status.setText("상태: 저장 완료")
        else:
            self.lbl_status.setText("상태: 저장 실패/데이터 없음")

    def on_open_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "파일 선택", BASE_SAVE_DIR, "YAML/JSON (*.yaml *.yml *.json)")
        if not path:
            return

        # 1) 기존 motions/waypoints 형식 먼저 시도
        seq = load_joint_seq_from_file(path)

        # 2) jointDataList 파일이면 자동 추출 → 변환 저장
        if not seq:
            # bridge 설정을 따라 축/반올림 적용
            axis_map = getattr(self.bridge, "joint_axis_map", ['y','x','x','x','y','x'])
            decimals = getattr(self.bridge, "joint_round_decimals", 2)
            seq = extract_joint_seq_from_jointdatalist_file(path, axis_map, decimals)
            if seq:
                # 변환 저장
                os.makedirs(BASE_SAVE_DIR, exist_ok=True)
                day = datetime.now().strftime("%Y%m%d")
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                day_dir = os.path.join(BASE_SAVE_DIR, day)
                os.makedirs(day_dir, exist_ok=True)
                base = os.path.splitext(os.path.basename(path))[0]
                name = f"{base}_conv_{ts}"
                out_path = os.path.join(day_dir, f"{name}.yaml")
                doc = {
                    "motions": [
                        {
                            "name": name,
                            "motion_type": "joint_path",
                            "execution_mode": "j_add",
                            "coordinate_frame": "robot_base_frame",
                            "parameters": {"wp_dt": float(self.spin_dt.value())},
                            "waypoints": [
                                {"id": i, "type": "joint", "joints": seq[i]}
                                for i in range(len(seq))
                            ],
                        }
                    ]
                }
                try:
                    if yaml:
                        with open(out_path, "w", encoding="utf-8") as f:
                            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
                    else:
                        out_path = out_path.replace(".yaml", ".json")
                        with open(out_path, "w", encoding="utf-8") as f:
                            json.dump(doc, f, ensure_ascii=False, indent=2)
                    self.lbl_file.setText(f"파일: {out_path} (변환 저장, N={len(seq)})")
                    self.lbl_status.setText("상태: jointDataList → joints 변환/저장 완료")
                except Exception as e:
                    self.lbl_status.setText(f"상태: 변환 저장 실패: {e}")
                    return
            else:
                self.lbl_status.setText("상태: 지원하지 않는 파일 형식")
                return

        # 로드 완료(재생/스텝 가능)
        self.seq = seq
        self.seq_idx = 0
        if not self.lbl_file.text().startswith("파일: "):
            self.lbl_file.setText(f"파일: {path} (N={len(seq)})")
        if "변환/저장" not in self.lbl_status.text():
            self.lbl_status.setText("상태: 파일 로드 완료")

    def on_play_start(self):
        if not self.seq:
            self.lbl_status.setText("상태: 재생할 데이터 없음")
            return
        self.bridge.playback_start(self.seq[self.seq_idx:] if self.seq_idx else self.seq, wp_dt=float(self.spin_dt.value()))
        self.lbl_status.setText("상태: 재생 중(ServoJ)")

    def on_play_stop(self):
        self.bridge.playback_stop()
        self.lbl_status.setText("상태: 재생 정지")

    def on_step_once(self):
        if self.seq is None or self.seq_idx >= len(self.seq):
            self.lbl_status.setText("상태: 스텝 전송할 데이터 없음")
            return
        q = self.seq[self.seq_idx]
        self.seq_idx += 1
        # 라이브 포워딩과 무관하게 1회 펄스 송신
        self.bridge.send_step_once(q, duration_s=0.2)
        self.lbl_status.setText(f"상태: 스텝 전송 {self.seq_idx}/{len(self.seq)}")

    def on_reset_index(self):
        self.seq_idx = 0
        self.lbl_status.setText("상태: 인덱스 0으로 리셋")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=10001)
    ap.add_argument("--robot-ip", default="192.168.1.13")
    ap.add_argument("--allow", default="192.168.56.1")
    args = ap.parse_args()

    bridge = UnityTcpBridge(
        robot_ip=args.robot_ip,
        listen_host=args.listen,
        listen_port=args.port,
        allowed_client_ip=args.allow,
        joint_axis_map=['y','x','x','x','y','x'],
        joint_round_decimals=2,
        servo_Ts=0.008,
        filter_alpha=1.0,
    )
    run_server_in_thread(bridge)

    app = QApplication([])
    win = TcpRecorderWindow(bridge)
    win.show()
    rc = app.exec_()

    bridge.playback_stop()
    bridge.stop()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()