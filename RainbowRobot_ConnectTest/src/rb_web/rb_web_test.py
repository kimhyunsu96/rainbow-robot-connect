#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
웹 기반 Rainbow Robot 모션 제어기
motion_gui_runner.py의 PyQt 기능을 Flask 웹으로 포팅

주요 기능:
1. Home 이동 (cobot.MoveJ with home_pose_arr)
2. Motion 파일 선택 및 관리 (~/motions 디렉터리)
3. ServoJ parameter 기반 모션 실행
"""

import os
import json
import threading
import time
from pathlib import Path
from datetime import datetime
import yaml
import sys

# Flask 웹서버
from flask import Flask, render_template, request, jsonify, send_file
from flask_cors import CORS

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Robot control
sys.path.insert(0, str(Path(__file__).parent.parent))
from rb_test import cobot
from rb_test.motion_executor import MotionExecutor

# ==================== 설정 ====================
def _guess_workspace_root() -> str:
    """workspace 루트 추론"""
    env = os.environ.get("SMART_WS_DIR")
    if env:
        return str(Path(env).expanduser().resolve())
    
    try:
        here = Path(__file__).resolve()
        parts = here.parts
        if "src" in parts:
            idx = parts.index("src")
            return str(Path(*parts[:idx]).resolve())
        if "install" in parts:
            idx = parts.index("install")
            return str(Path(*parts[:idx]).resolve())
    except Exception:
        pass
    
    return str(Path.cwd().resolve())


def _default_motions_dir() -> str:
    """motions 디렉터리 경로"""
    ws = Path(_guess_workspace_root())
    return str((ws / "motions").resolve())


# ==================== ROS2 Node ====================
class MotionWebNode(Node):
    def __init__(self):
        super().__init__('motion_web_runner')
        
        # 파라미터 선언
        self.declare_parameter('home_pose_arr', [0.0, -45.00, 137.00, 0.00, -90.00, 0.0])
        self.declare_parameter('home_speed', 20.0)
        self.declare_parameter('home_accel', 20.0)
        
        self.declare_parameter('unity_playback_servo_t1', 0.05)
        self.declare_parameter('unity_playback_servo_t2', 0.05)
        self.declare_parameter('unity_playback_servo_gain', 0.1)
        self.declare_parameter('unity_playback_servo_alpha', 0.03)
        
        self.declare_parameter('default_dir', _default_motions_dir())
        self.declare_parameter('run_mode', 'inline')  # 'inline' | 'process'
        
        # 파라미터 로드
        self.home_pose_arr = list(self.get_parameter('home_pose_arr').value)
        self.home_speed = float(self.get_parameter('home_speed').value)
        self.home_accel = float(self.get_parameter('home_accel').value)
        
        self.servo_params = {
            't1': float(self.get_parameter('unity_playback_servo_t1').value),
            't2': float(self.get_parameter('unity_playback_servo_t2').value),
            'gain': float(self.get_parameter('unity_playback_servo_gain').value),
            'alpha': float(self.get_parameter('unity_playback_servo_alpha').value),
        }
        
        self.default_dir = self.get_parameter('default_dir').value
        self.run_mode = self.get_parameter('run_mode').value
        
        # 상태
        self._busy = False
        self._selected_motion_file = None
        self._last_status = ""
        self._motion_duration_s = None
        
        # Inline 모드용 MotionExecutor
        self.exec_node = None
        if self.run_mode == 'inline':
            try:
                self.exec_node = MotionExecutor()
                self.exec_node.set_servo_overrides(**self.servo_params)
                self.get_logger().info("[INIT] MotionExecutor initialized")
            except Exception as e:
                self.get_logger().warning(f"[INIT] MotionExecutor init failed: {e}")
        
        # Duration 구독
        self.duration_sub = self.create_subscription(
            Float32, "/motion_executor/last_duration_s",
            self._on_motion_duration, 10
        )
        
        self.get_logger().info("[INIT] MotionWebNode initialized")
    
    def _on_motion_duration(self, msg: Float32):
        """모션 실행 시간 수신"""
        self._motion_duration_s = float(msg.data)
    
    def get_status(self) -> str:
        """현재 상태 반환"""
        status = {
            'busy': self._busy,
            'selected_file': self._selected_motion_file,
            'last_status': self._last_status,
            'servo_params': self.servo_params,
            'home_pose': self.home_pose_arr,
        }
        return status
    
    def set_status(self, msg: str):
        """상태 메시지 업데이트"""
        self._last_status = msg
        self.get_logger().info(f"[STATUS] {msg}")
    
    def is_busy(self) -> bool:
        """실행 중 여부"""
        return self._busy
    
    def run_home(self) -> bool:
        """Home 위치로 이동"""
        if self._busy:
            self.set_status("실행 중 → 무시")
            return False
        
        self._busy = True
        try:
            def _do():
                try:
                    q = self.home_pose_arr
                    sp = self.home_speed
                    ac = self.home_accel
                    
                    self.set_status(f"홈 이동: {q} (speed={sp}, accel={ac})")
                    
                    # cobot.MoveJ 호출
                    cobot.MoveJ(q[0], q[1], q[2], q[3], q[4], q[5], sp, ac)
                    self.set_status("홈 이동 완료")
                    
                except Exception as e:
                    self.set_status(f"홈 이동 실패: {e}")
                    self.get_logger().error(f"[HOME] failed: {e}")
                finally:
                    self._busy = False
            
            threading.Thread(target=_do, daemon=True).start()
            return True
        except Exception as e:
            self._busy = False
            self.set_status(f"홈 이동 시작 실패: {e}")
            return False
    
    def load_motion_file(self, filepath: str) -> bool:
        """모션 파일 선택 및 저장"""
        if not filepath or not os.path.exists(filepath):
            self.set_status(f"파일을 찾을 수 없음: {filepath}")
            return False
        
        ext = Path(filepath).suffix.lower()
        if ext not in ['.yaml', '.yml', '.json']:
            self.set_status(f"지원하지 않는 파일 형식: {ext}")
            return False
        
        self._selected_motion_file = filepath
        self.set_status(f"모션 파일 로드됨: {Path(filepath).name}")
        return True
    
    def run_motion_file(self, filepath: str = None) -> bool:
        """선택한 모션 파일 실행 (ServoJ parameter 사용)"""
        if self._busy:
            self.set_status("실행 중 → 무시")
            return False
        
        motion_file = filepath or self._selected_motion_file
        if not motion_file or not os.path.exists(motion_file):
            self.set_status("모션 파일이 선택되지 않았습니다")
            return False
        
        self._busy = True
        try:
            def _do():
                try:
                    if self.exec_node is not None:
                        # Inline 모드: MotionExecutor 사용
                        self.exec_node.load_motion_from_file(motion_file, False)
                        self.set_status("모션 실행 완료 (inline)")
                    else:
                        # Process 모드: 외부 실행 (구현 필요)
                        self.set_status("Process 모드는 현재 미지원")
                    
                except Exception as e:
                    self.set_status(f"모션 실행 실패: {e}")
                    self.get_logger().error(f"[MOTION] failed: {e}")
                finally:
                    self._busy = False
            
            threading.Thread(target=_do, daemon=True).start()
            return True
        except Exception as e:
            self._busy = False
            self.set_status(f"모션 실행 시작 실패: {e}")
            return False
    
    def set_servo_params(self, t1=None, t2=None, gain=None, alpha=None):
        """ServoJ 파라미터 설정"""
        if t1 is not None:
            self.servo_params['t1'] = float(t1)
        if t2 is not None:
            self.servo_params['t2'] = float(t2)
        if gain is not None:
            self.servo_params['gain'] = float(gain)
        if alpha is not None:
            self.servo_params['alpha'] = float(alpha)
        
        # MotionExecutor에 반영
        if self.exec_node is not None:
            try:
                self.exec_node.set_servo_overrides(**self.servo_params)
            except Exception as e:
                self.get_logger().warning(f"servo override failed: {e}")
        
        self.set_status(f"ServoJ 파라미터 업데이트: {self.servo_params}")


# ==================== Flask 웹 앱 ====================
app = Flask(__name__)
CORS(app)

# 전역 ROS2 노드
ros_node = None
executor_thread = None


def init_ros():
    """ROS2 초기화"""
    global ros_node, executor_thread
    
    rclpy.init()
    ros_node = MotionWebNode()
    
    def run_executor():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(ros_node)
        executor.spin()
    
    executor_thread = threading.Thread(target=run_executor, daemon=True)
    executor_thread.start()


@app.route('/')
def index():
    """메인 페이지"""
    return render_template('index.html')


@app.route('/api/status', methods=['GET'])
def get_status():
    """현재 상태 반환"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    status = ros_node.get_status()
    return jsonify(status)


@app.route('/api/motions-list', methods=['GET'])
def list_motions():
    """motions 디렉터리의 파일 목록"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    motions_dir = Path(ros_node.default_dir)
    if not motions_dir.exists():
        return jsonify({'error': f'Directory not found: {motions_dir}'}), 404
    
    files = []
    for ext in ['*.yaml', '*.yml', '*.json']:
        files.extend(motions_dir.glob(ext))
    
    # 서브디렉터리도 검색
    for ext in ['**/*.yaml', '**/*.yml', '**/*.json']:
        files.extend(motions_dir.glob(ext))
    
    files = sorted(set(files))
    file_list = [
        {
            'name': f.name,
            'path': str(f.resolve()),
            'relative': str(f.relative_to(motions_dir)),
            'size': f.stat().st_size,
            'modified': datetime.fromtimestamp(f.stat().st_mtime).isoformat(),
        }
        for f in files
    ]
    
    return jsonify({'files': file_list, 'directory': str(motions_dir)})


@app.route('/api/load-motion', methods=['POST'])
def load_motion():
    """모션 파일 선택"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.get_json()
    filepath = data.get('filepath')
    
    if not filepath:
        return jsonify({'error': 'filepath required'}), 400
    
    success = ros_node.load_motion_file(filepath)
    return jsonify({
        'success': success,
        'selected_file': ros_node._selected_motion_file,
        'status': ros_node._last_status,
    })


@app.route('/api/run-home', methods=['POST'])
def run_home():
    """Home 이동 실행"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    success = ros_node.run_home()
    return jsonify({
        'success': success,
        'busy': ros_node.is_busy(),
        'status': ros_node._last_status,
    })


@app.route('/api/run-motion', methods=['POST'])
def run_motion():
    """선택한 모션 파일 실행"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    data = request.get_json() or {}
    filepath = data.get('filepath')
    
    success = ros_node.run_motion_file(filepath)
    return jsonify({
        'success': success,
        'busy': ros_node.is_busy(),
        'status': ros_node._last_status,
    })


@app.route('/api/servo-params', methods=['GET', 'POST'])
def servo_params():
    """ServoJ 파라미터 조회/수정"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    if request.method == 'GET':
        return jsonify(ros_node.servo_params)
    
    # POST: 파라미터 업데이트
    data = request.get_json() or {}
    ros_node.set_servo_params(
        t1=data.get('t1'),
        t2=data.get('t2'),
        gain=data.get('gain'),
        alpha=data.get('alpha'),
    )
    
    return jsonify({
        'success': True,
        'servo_params': ros_node.servo_params,
        'status': ros_node._last_status,
    })


@app.route('/api/home-pose', methods=['GET', 'POST'])
def home_pose():
    """Home 위치 조회/수정"""
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    
    if request.method == 'GET':
        return jsonify({
            'home_pose': ros_node.home_pose_arr,
            'home_speed': ros_node.home_speed,
            'home_accel': ros_node.home_accel,
        })
    
    # POST: 업데이트
    data = request.get_json() or {}
    if 'home_pose' in data:
        ros_node.home_pose_arr = list(data['home_pose'])
    if 'home_speed' in data:
        ros_node.home_speed = float(data['home_speed'])
    if 'home_accel' in data:
        ros_node.home_accel = float(data['home_accel'])
    
    return jsonify({
        'success': True,
        'home_pose': ros_node.home_pose_arr,
        'home_speed': ros_node.home_speed,
        'home_accel': ros_node.home_accel,
    })


if __name__ == '__main__':
    print("[INIT] Starting Rainbow Robot Web Control...")
    init_ros()
    
    # Flask 실행
    app.run(
        host='0.0.0.0',
        port=5000,
        debug=False,
        use_reloader=False,
    )
