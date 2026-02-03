#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
웹 기반 Rainbow Robot 모션 제어기 (개선 버전)
- ROS2 없어도 웹 서버 실행 가능
- 로봇 연결은 백그라운드에서 비동기 처리 (블로킹 없음)
"""

import os
import json
import threading
import time
import signal
from pathlib import Path
from datetime import datetime
import yaml
import sys

# Flask 웹서버
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS

# ROS2 및 로봇 제어 모듈 (선택사항)
HAS_ROS2 = False
HAS_ROBOT = False

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    HAS_ROS2 = True
except ImportError:
    pass

try:
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from rb_test import cobot
    from rb_test.motion_executor import MotionExecutor
    HAS_ROBOT = True
except ImportError:
    pass

# ==================== Flask App ====================
app = Flask(__name__, template_folder='templates', static_folder='static')
CORS(app)

# ==================== 전역 변수 ====================
ros_node = None  # ROS2 노드

class DummyNode:
    """ROS2 없을 때 사용하는 더미 노드"""
    def __init__(self):
        self.home_pose_arr = [0.0, -45.00, 137.00, 0.00, -90.00, 0.0]
        self.home_speed = 20.0
        self.home_accel = 20.0
        self.servo_params = {
            't1': 0.05, 't2': 0.05, 'gain': 0.1, 'alpha': 0.03
        }
        self.default_dir = _get_default_motions_dir()
        
        self._busy = False
        self._selected_motion_file = None
        self._last_status = "웹 서버 실행 중 (로봇 미연결)"
        self._motion_duration_s = None
        self._robot_connected = False
        self._robot_error_msg = "ROS2/로봇 모듈 미설치"
        
        self.exec_node = None
    
    def is_busy(self):
        return self._busy
    
    def get_status_dict(self):
        return {
            'busy': self._busy,
            'selected_file': self._selected_motion_file,
            'status': self._last_status,
            'duration': self._motion_duration_s,
            'robot_connected': self._robot_connected,
            'robot_error': self._robot_error_msg,
            'servo_params': self.servo_params,
            'home_pose': self.home_pose_arr,
            'home_speed': self.home_speed,
            'home_accel': self.home_accel,
        }
    
    def load_motion_file(self, filepath):
        return False
    
    def run_home(self):
        return False
    
    def run_motion_file(self, filepath):
        return False
    
    def set_servo_params(self, **kwargs):
        pass

def _get_default_motions_dir():
    """motions 디렉터리 찾기"""
    # 현재 파일의 위치에서 RainbowRobot_ConnectTest/motions를 찾기
    current_file = Path(__file__).resolve()
    
    # src/rb_web에서 시작해서 상위 디렉터리로 이동
    rb_web_dir = current_file.parent
    src_dir = rb_web_dir.parent
    project_root = src_dir.parent
    
    # RainbowRobot_ConnectTest/motions 경로 확인
    motions_dir = project_root / "RainbowRobot_ConnectTest" / "motions"
    
    if motions_dir.exists():
        return str(motions_dir)
    
    # 대안 경로들 시도
    alt_paths = [
        Path.home() / "motions",
        Path.cwd() / "motions",
        project_root / "motions",
    ]
    
    for path in alt_paths:
        if path.exists():
            return str(path)
    
    # 기본값
    return str(Path.home() / "motions")

# ==================== ROS2 Node 정의 ====================
if HAS_ROS2:
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
            self.declare_parameter('default_dir', _get_default_motions_dir())
            self.declare_parameter('run_mode', 'inline')
            
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
            self._last_status = "준비됨"
            self._motion_duration_s = None
            self._robot_connected = False
            self._robot_error_msg = "연결 중..."
            self.exec_node = None
            
            # 백그라운드에서 로봇 초기화
            self._init_robot_in_background()
            
            # Duration 구독
            self.duration_sub = self.create_subscription(
                Float32, "/motion_executor/last_duration_s",
                self._on_motion_duration, 10
            )
            
            self.get_logger().info("[INIT] MotionWebNode initialized")
        
        def _init_robot_in_background(self):
            """백그라운드에서 로봇 초기화 (타임아웃 10초)"""
            def _init():
                if self.run_mode != 'inline' or not HAS_ROBOT:
                    self._robot_connected = False
                    self._robot_error_msg = "로봇 제어 미지원"
                    return
                
                try:
                    self.get_logger().info("[ROBOT] Connecting to robot (timeout: 10s)...")
                    
                    # 타임아웃 핸들러
                    def timeout_handler(signum, frame):
                        raise TimeoutError("Robot connection timeout (10s)")
                    
                    signal.signal(signal.SIGALRM, timeout_handler)
                    signal.alarm(10)
                    
                    try:
                        # 1️⃣ MotionExecutor 생성 시도
                        self.exec_node = MotionExecutor()
                        self.get_logger().info("[ROBOT] MotionExecutor created")
                        
                        # 2️⃣ 실제 로봇과 통신 테스트 (핸드셰이크)
                        # set_servo_overrides()가 실패하면 로봇이 응답하지 않음
                        self.exec_node.set_servo_overrides(**self.servo_params)
                        self.get_logger().info("[ROBOT] Servo parameters set successfully")
                        
                        # 3️⃣ 추가 확인: 로봇 상태 읽기 시도
                        # (MotionExecutor 객체에서 상태를 읽을 수 있으면 통신 확인)
                        try:
                            # 로봇 상태 확인 (예: 현재 위치 읽기)
                            if hasattr(self.exec_node, 'get_joint_values'):
                                joint_values = self.exec_node.get_joint_values()
                                self.get_logger().info(f"[ROBOT] Joint values: {joint_values}")
                            elif hasattr(self.exec_node, 'get_current_state'):
                                state = self.exec_node.get_current_state()
                                self.get_logger().info(f"[ROBOT] Robot state: {state}")
                        except Exception as state_check_error:
                            self.get_logger().warning(f"[ROBOT] Could not read robot state: {state_check_error}")
                            # 상태 읽기 실패는 치명적이지 않음
                        
                        # ✅ 모든 체크 통과
                        self._robot_connected = True
                        self._robot_error_msg = ""
                        self._last_status = "✅ 로봇 연결됨"
                        self.get_logger().info("[ROBOT] ✅ Robot connected and verified!")
                        
                    finally:
                        signal.alarm(0)
                
                except TimeoutError:
                    self._robot_connected = False
                    self._robot_error_msg = "연결 시간 초과 (10초) - 로봇 응답 없음"
                    self._last_status = f"⚠️ 로봇 미연결: {self._robot_error_msg}"
                    self.get_logger().warning(f"[ROBOT] ⚠️ {self._last_status}")
                    
                except Exception as e:
                    self._robot_connected = False
                    error_str = str(e)
                    
                    # 더 자세한 오류 정보 기록
                    if "Connection refused" in error_str or "110" in error_str:
                        self._robot_error_msg = "로봇 IP (192.168.1.13) 응답 없음"
                    elif "Motor" in error_str or "servo" in error_str.lower():
                        self._robot_error_msg = "로봇 하드웨어 오류"
                    elif "timeout" in error_str.lower():
                        self._robot_error_msg = "네트워크 연결 시간 초과"
                    else:
                        self._robot_error_msg = error_str[:50]
                    
                    self._last_status = f"⚠️ 로봇 미연결: {self._robot_error_msg}"
                    self.get_logger().warning(f"[ROBOT] ⚠️ Connection failed: {e}")
            
            thread = threading.Thread(target=_init, daemon=True)
            thread.start()
        
        def _on_motion_duration(self, msg):
            self._motion_duration_s = float(msg.data)
        
        def is_busy(self):
            return self._busy
        
        def get_status_dict(self):
            return {
                'busy': self._busy,
                'selected_file': self._selected_motion_file,
                'status': self._last_status,
                'duration': self._motion_duration_s,
                'robot_connected': self._robot_connected,
                'robot_error': self._robot_error_msg,
                'servo_params': self.servo_params,
                'home_pose': self.home_pose_arr,
                'home_speed': self.home_speed,
                'home_accel': self.home_accel,
            }
        
        def set_status(self, msg):
            self._last_status = msg
            self.get_logger().info(f"[STATUS] {msg}")
        
        def load_motion_file(self, filepath):
            if not filepath:
                self.set_status("❌ 파일 경로 필요")
                return False
            
            try:
                self._selected_motion_file = filepath
                self.set_status(f"✅ 모션 파일 로드: {Path(filepath).name}")
                return True
            except Exception as e:
                self.set_status(f"❌ 파일 로드 실패: {e}")
                return False
        
        def run_home(self):
            if self._busy:
                self.set_status("⏳ 현재 다른 작업 중...")
                return False
            
            if not self._robot_connected:
                self.set_status(f"❌ 로봇 미연결: {self._robot_error_msg}")
                return False
            
            self._busy = True
            try:
                self.set_status("🏠 홈 위치로 이동 중...")
                if HAS_ROBOT:
                    cobot.MoveJ(self.home_pose_arr, self.home_speed, self.home_accel)
                    self.set_status("✅ 홈 위치 이동 완료")
                return True
            except Exception as e:
                self.set_status(f"❌ 홈 이동 실패: {e}")
                return False
            finally:
                self._busy = False
        
        def run_motion_file(self, filepath=None):
            if self._busy:
                self.set_status("⏳ 현재 다른 작업 중...")
                return False
            
            if not self._robot_connected:
                self.set_status(f"❌ 로봇 미연결: {self._robot_error_msg}")
                return False
            
            filepath = filepath or self._selected_motion_file
            if not filepath:
                self.set_status("❌ 모션 파일 선택 필요")
                return False
            
            self._busy = True
            try:
                self.set_status(f"▶️ 모션 실행: {Path(filepath).name}...")
                if self.run_mode == 'inline' and self.exec_node:
                    self.exec_node.execute_motion(
                        filepath, 
                        inline=True,
                        servo_override=self.servo_params
                    )
                    self.set_status("✅ 모션 실행 완료")
                return True
            except Exception as e:
                self.set_status(f"❌ 모션 실행 실패: {e}")
                return False
            finally:
                self._busy = False
        
        def set_servo_params(self, t1=None, t2=None, gain=None, alpha=None):
            if t1 is not None:
                self.servo_params['t1'] = float(t1)
            if t2 is not None:
                self.servo_params['t2'] = float(t2)
            if gain is not None:
                self.servo_params['gain'] = float(gain)
            if alpha is not None:
                self.servo_params['alpha'] = float(alpha)
            
            if self.exec_node:
                try:
                    self.exec_node.set_servo_overrides(**self.servo_params)
                except:
                    pass

# ==================== Flask Routes ====================
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def status():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    return jsonify(ros_node.get_status_dict())

@app.route('/api/motions-list', methods=['GET'])
def motions_list():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    motions_dir = Path(ros_node.default_dir)
    if not motions_dir.exists():
        return jsonify({'files': [], 'directory': str(motions_dir)})
    
    files = [f for f in motions_dir.glob('*.yaml')]
    files += [f for f in motions_dir.glob('*.json')]
    
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
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    data = request.get_json()
    filepath = data.get('filepath')
    
    if not filepath:
        return jsonify({'error': 'filepath required'}), 400
    
    success = ros_node.load_motion_file(filepath)
    status_dict = ros_node.get_status_dict()
    return jsonify({'success': success, **status_dict})

@app.route('/api/run-home', methods=['POST'])
def run_home():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    success = ros_node.run_home()
    status_dict = ros_node.get_status_dict()
    return jsonify({'success': success, **status_dict})

@app.route('/api/run-motion', methods=['POST'])
def run_motion():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    data = request.get_json() or {}
    filepath = data.get('filepath')
    
    success = ros_node.run_motion_file(filepath)
    status_dict = ros_node.get_status_dict()
    return jsonify({'success': success, **status_dict})

@app.route('/api/servo-params', methods=['GET', 'POST'])
def servo_params():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    if request.method == 'GET':
        return jsonify(ros_node.servo_params)
    
    data = request.get_json() or {}
    ros_node.set_servo_params(
        t1=data.get('t1'),
        t2=data.get('t2'),
        gain=data.get('gain'),
        alpha=data.get('alpha'),
    )
    
    status_dict = ros_node.get_status_dict()
    return jsonify({'success': True, **status_dict})

@app.route('/api/home-pose', methods=['GET', 'POST'])
def home_pose():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    if request.method == 'GET':
        return jsonify({
            'home_pose': ros_node.home_pose_arr,
            'home_speed': ros_node.home_speed,
            'home_accel': ros_node.home_accel,
        })
    
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

@app.route('/api/robot-diagnostics', methods=['GET'])
def robot_diagnostics():
    """🔍 로봇 연결 상태 진단"""
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    diagnostics = {
        'robot_connected': ros_node._robot_connected,
        'robot_error': ros_node._robot_error_msg,
        'status': ros_node._last_status,
        'has_executor': ros_node.exec_node is not None,
        'executor_type': type(ros_node.exec_node).__name__ if ros_node.exec_node else None,
    }
    
    # 로봇이 연결되었다고 표시되면 실제 상태 확인
    if ros_node._robot_connected and ros_node.exec_node:
        try:
            # MotionExecutor의 메서드 확인
            diagnostics['executor_methods'] = [m for m in dir(ros_node.exec_node) if not m.startswith('_')]
            
            # 로봇 상태 읽기 시도
            if hasattr(ros_node.exec_node, 'get_joint_values'):
                try:
                    joint_values = ros_node.exec_node.get_joint_values()
                    diagnostics['joint_values'] = joint_values
                    diagnostics['connection_verified'] = True
                except Exception as e:
                    diagnostics['joint_values_error'] = str(e)
                    diagnostics['connection_verified'] = False
            
            if hasattr(ros_node.exec_node, 'get_current_state'):
                try:
                    state = ros_node.exec_node.get_current_state()
                    diagnostics['robot_state'] = str(state)
                    diagnostics['connection_verified'] = True
                except Exception as e:
                    diagnostics['robot_state_error'] = str(e)
                    diagnostics['connection_verified'] = False
                    
        except Exception as e:
            diagnostics['diagnostic_error'] = str(e)
    
    return jsonify(diagnostics)

# ==================== Main ====================
if __name__ == '__main__':
    print(r"""
                                    
      ,--.                     ,--. 
,--.--|  |-.,-----.,--,--.,---.`--' 
|  .--| .-. '-----' ,-.  | .-. ,--. 
|  |  | `-' |     \ '-'  | '-' |  | 
`--'   `---'       `--`--|  |-'`--' 
                         `--'       
    Rainbow Robot Web Control
""")
    
    print("[INIT] Starting Rainbow Robot Web Control...")
    
    # ROS2/로봇 초기화 시도
    if HAS_ROS2:
        print("[INIT] ROS2 detected - initializing...")
        try:
            rclpy.init()
            ros_node = MotionWebNode()
            
            # 백그라운드에서 ROS2 스핀
            def ros2_spin():
                try:
                    rclpy.spin(ros_node)
                except KeyboardInterrupt:
                    pass
                except Exception as e:
                    print(f"[ROS2] Error: {e}")
                finally:
                    try:
                        if ros_node:
                            ros_node.destroy_node()
                        rclpy.shutdown()
                    except:
                        pass
            
            ros_thread = threading.Thread(target=ros2_spin, daemon=True)
            ros_thread.start()
            
            print("[INIT] ✅ ROS2 node created")
            print("[ROBOT] Robot connection attempt in background (timeout: 10s)...")
            
        except Exception as e:
            print(f"[ERROR] ROS2 init failed: {e}")
            print("[INFO] Using dummy node for standalone web server...")
            ros_node = DummyNode()
    else:
        print("[WARN] ROS2 not installed")
        print("[INFO] Using dummy node for standalone web server...")
        ros_node = DummyNode()
    
    # Flask 웹 서버 실행
    print("\n[WEB] Starting Flask web server...")
    print("[WEB] Open browser at: http://localhost:5000")
    print("[WEB] Press Ctrl+C to stop\n")
    
    # 백그라운드에서 서버 실행
    from werkzeug.serving import make_server
    import threading
    
    def run_server():
        try:
            server = make_server('0.0.0.0', 5000, app, threaded=True)
            server.serve_forever()
        except Exception as e:
            print(f"[WEB] Server error: {e}")
    
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    
    try:
        # 메인 스레드에서 대기
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Stopping web server...")
    except Exception as e:
        print(f"[ERROR] Web server error: {e}")
