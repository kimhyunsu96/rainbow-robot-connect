# Rainbow Robot 웹 제어기 - 구현 가이드

## 📋 프로젝트 구조

### 백엔드 (Flask + ROS2)
```
rb_web_test.py
├── MotionWebNode (ROS2 Node)
│   ├── run_home()           - Home 위치로 이동
│   ├── load_motion_file()   - 모션 파일 선택
│   ├── run_motion_file()    - 선택한 모션 실행
│   └── set_servo_params()   - ServoJ 파라미터 설정
│
└── Flask App
    ├── /api/status          - 상태 조회
    ├── /api/motions-list    - 파일 목록
    ├── /api/load-motion     - 파일 선택
    ├── /api/run-home        - Home 실행
    ├── /api/run-motion      - 모션 실행
    ├── /api/servo-params    - 파라미터 관리
    └── /api/home-pose       - Home 위치 관리
```

### 프론트엔드 (HTML + CSS + JS)
```
templates/index.html  - 웹 페이지 구조
static/
├── style.css         - 스타일 (반응형 디자인)
└── script.js         - 클라이언트 로직
```

## 🔧 motion_gui_runner.py에서 포팅된 부분

### 1. Home 이동 기능
**원본 코드 (motion_gui_runner.py, 라인 1816-1885):**
```python
def run_home(self) -> bool:
    # cobot.MoveJ로 home_pose_arr 실행
    q = self.home_pose_arr
    cobot.MoveJ(q[0], q[1], q[2], q[3], q[4], q[5], 
                self.home_speed, self.home_accel)
```

**웹 버전 (rb_web_test.py):**
```python
def run_home(self) -> bool:
    """Home 위치로 이동"""
    def _do():
        q = self.home_pose_arr
        cobot.MoveJ(q[0], q[1], q[2], q[3], q[4], q[5], 
                   self.home_speed, self.home_accel)
    threading.Thread(target=_do, daemon=True).start()
```

### 2. 모션 파일 검색 및 관리
**원본 코드:**
```python
def _default_motions_dir() -> str:
    ws = Path(_guess_workspace_root())
    return str((ws / "motions").resolve())
```

**웹 버전 (API):**
```python
@app.route('/api/motions-list', methods=['GET'])
def list_motions():
    motions_dir = Path(ros_node.default_dir)
    files = []
    for ext in ['*.yaml', '*.yml', '*.json']:
        files.extend(motions_dir.glob(ext))
    # 재귀 검색
    for ext in ['**/*.yaml', '**/*.yml', '**/*.json']:
        files.extend(motions_dir.glob(ext))
    return jsonify({'files': file_list})
```

### 3. ServoJ 파라미터 관리
**원본 코드:**
```python
self.servo_ui = {
    't1': 0.08, 
    't2': 0.05, 
    'gain': 0.05, 
    'alpha': 0.003
}

def set_servo_overrides(self, t1=None, t2=None, gain=None, alpha=None):
    if t1 is not None: self.servo_ui['t1'] = float(t1)
    # ... 나머지 파라미터
```

**웹 버전:**
```python
def set_servo_params(self, t1=None, t2=None, gain=None, alpha=None):
    if t1 is not None:
        self.servo_params['t1'] = float(t1)
    # MotionExecutor에 반영
    if self.exec_node is not None:
        self.exec_node.set_servo_overrides(**self.servo_params)
```

### 4. 모션 파일 실행
**원본 코드:**
```python
def run_motion_file(self, motion_file: str) -> bool:
    if self.exec_node is not None:
        self.exec_node.load_motion_from_file(motion_file, False)
```

**웹 버전:**
```python
def run_motion_file(self, filepath: str = None) -> bool:
    def _do():
        if self.exec_node is not None:
            self.exec_node.load_motion_from_file(motion_file, False)
    threading.Thread(target=_do, daemon=True).start()
```

## 🚀 주요 개선 사항

### 1. 비동기 실행
- 모든 로봇 제어 작업을 백그라운드 스레드에서 실행
- UI 블로킹 없음

### 2. RESTful API 제공
- 표준화된 HTTP 인터페이스
- JSON 기반 통신
- 프론트엔드에서 쉬운 통합

### 3. 반응형 웹 UI
- 모바일/태블릿 지원
- 직관적인 인터페이스
- 실시간 상태 업데이트

### 4. 오류 처리 개선
- 예외 상황 명확한 알림
- 로그 시스템 통합
- 사용자 친화적 메시지

## 📱 웹 UI 기능 상세

### 상태 섹션
```
┌─────────────────────────────────────┐
│ 시스템 상태                          │
├─────────────────────────────────────┤
│ 상태: 준비됨                         │
│ 실행 중: 아니오                      │
│ 선택 파일: home_motion.yaml          │
└─────────────────────────────────────┘
```

### 파일 관리 섹션
```
┌─────────────────────────────────────┐
│ 모션 파일 선택                       │
├─────────────────────────────────────┤
│ [🔄 파일 목록 새로고침]              │
│                                      │
│ 📄 home_motion.yaml     [선택]      │
│ 📄 standby_motion.yaml  [선택]      │
│ 📄 motion_20250925.yaml [선택]      │
└─────────────────────────────────────┘
```

### Home 제어 섹션
```
┌─────────────────────────────────────┐
│ Home 이동                            │
├─────────────────────────────────────┤
│ Home 위치: [0.0, -45.0, 137.0, ...]  │
│ [입력 필드]                [설정]   │
│                                      │
│ 이동 속도: [20.0    ]                 │
│ 가속도:    [20.0    ]                 │
│                                      │
│        [🚀 Home으로 이동]             │
└─────────────────────────────────────┘
```

### ServoJ 파라미터 섹션
```
┌─────────────────────────────────────┐
│ ServoJ 파라미터                      │
├─────────────────────────────────────┤
│ t1 (Time constant 1): [0.05  ]       │
│ t2 (Time constant 2): [0.05  ]       │
│ Gain (제어 이득):      [0.1   ]       │
│ Alpha (필터계수):      [0.03  ]       │
│                                      │
│  [📝 ServoJ 파라미터 업데이트]       │
│                                      │
│ 현재 값:                             │
│ {                                    │
│   "t1": 0.05,                        │
│   "t2": 0.05,                        │
│   "gain": 0.1,                       │
│   "alpha": 0.03                      │
│ }                                    │
└─────────────────────────────────────┘
```

### 모션 실행 섹션
```
┌─────────────────────────────────────┐
│ 모션 실행                            │
├─────────────────────────────────────┤
│ 선택된 모션: home_motion.yaml         │
│                                      │
│       [▶️ 모션 실행]                  │
└─────────────────────────────────────┘
```

## 💡 사용 예시

### 1. 간단한 Home 이동
```bash
# 브라우저에서
1. http://localhost:5000 접속
2. "Home으로 이동" 버튼 클릭
3. 로그에서 진행 상황 확인
```

### 2. 새로운 모션 파일 로드 및 실행
```bash
1. 파일 목록에서 "motion_20250925.yaml" 선택
2. ServoJ 파라미터 필요시 조정
3. "모션 실행" 버튼 클릭
```

### 3. API를 통한 프로그래매틱 제어
```python
import requests

# Home 이동
response = requests.post('http://localhost:5000/api/run-home')

# 모션 파일 선택 및 실행
requests.post('http://localhost:5000/api/load-motion', 
              json={'filepath': '/workspace/motions/motion.yaml'})
requests.post('http://localhost:5000/api/run-motion')

# ServoJ 파라미터 업데이트
requests.post('http://localhost:5000/api/servo-params',
              json={'t1': 0.08, 't2': 0.06, 'gain': 0.15, 'alpha': 0.05})
```

## 🔍 디버깅 팁

### 서버 로그 확인
```bash
# 터미널에 출력되는 로그 확인
python3 rb_web_test.py
```

### 웹 브라우저 개발자 도구
```
F12 → Console 탭
- API 호출 로그 확인
- 에러 메시지 확인
```

### API 테스트 (curl)
```bash
# 상태 조회
curl http://localhost:5000/api/status

# 파일 목록
curl http://localhost:5000/api/motions-list

# Home 실행
curl -X POST http://localhost:5000/api/run-home
```

## 📝 설정 파일

### launch 파일 예시 (ROS2)
```yaml
# motion_web.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rb_test',
            executable='rb_web_test',
            name='motion_web_runner',
            parameters=[
                {'home_pose_arr': [0.0, -45.00, 137.00, 0.00, -90.00, 0.0]},
                {'home_speed': 20.0},
                {'home_accel': 20.0},
                {'unity_playback_servo_t1': 0.05},
                {'unity_playback_servo_t2': 0.05},
                {'unity_playback_servo_gain': 0.1},
                {'unity_playback_servo_alpha': 0.03},
                {'run_mode': 'inline'},
            ]
        )
    ])
```

## 🎯 다음 단계 (확장 기능)

1. **WebSocket 지원**
   - 실시간 양방향 통신
   - 더 부드러운 스트리밍

2. **파일 업로드/다운로드**
   - 웹에서 모션 파일 편집
   - 새 모션 파일 생성

3. **모션 플레이백 시뮬레이션**
   - 3D 시각화
   - 경로 미리보기

4. **로그 저장 및 재생**
   - 실행 이력 저장
   - 문제 분석

5. **다중 로봇 지원**
   - 여러 로봇 동시 제어
   - 로봇별 인터페이스

## 📞 문제 해결 가이드

| 증상 | 원인 | 해결책 |
|------|------|--------|
| 웹 페이지 로드 실패 | Flask 서버 미실행 | `python3 rb_web_test.py` 실행 |
| 모션 파일이 없음 | motions 폴더 경로 오류 | `SMART_WS_DIR` 환경변수 설정 |
| Home 이동 실패 | cobot 모듈 오류 | ROS2 초기화 확인 |
| ServoJ 적용 안 됨 | MotionExecutor 미초기화 | `run_mode: inline` 확인 |
| CORS 오류 | 브라우저 정책 | Flask-CORS 설치 확인 |

---

**작성자**: Rainbow Robot 팀  
**작성일**: 2025-01-22  
**버전**: 1.0
