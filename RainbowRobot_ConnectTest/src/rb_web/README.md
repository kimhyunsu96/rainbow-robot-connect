# Rainbow Robot 웹 제어기

## 개요

`rb_web_test.py`는 PyQt 기반의 `motion_gui_runner.py`의 핵심 기능을 Flask 웹 애플리케이션으로 포팅한 것입니다.

웹 브라우저를 통해 Rainbow Robot을 제어하고 모션 파일을 관리할 수 있습니다.

## 주요 기능

### 1. 🏠 Home 이동
- `cobot.MoveJ`를 사용하여 로봇을 홈 위치로 이동
- `home_pose_arr` 파라미터로 홈 위치 설정 가능
- 이동 속도(`home_speed`)와 가속도(`home_accel`) 조절 가능

### 2. 📁 모션 파일 관리
- `~/motions` 디렉터리에서 YAML/JSON 파일 자동 검색
- 파일 목록 보기 및 선택
- 선택한 파일 정보 저장

### 3. ⚙️ ServoJ 파라미터 제어
- `t1`, `t2`, `gain`, `alpha` 파라미터 조절
- 웹 UI에서 실시간 업데이트

### 4. ▶️ 모션 실행
- 선택한 모션 파일을 ServoJ 파라미터로 실행
- `MotionExecutor`를 통한 inline 모드 지원
- 실행 상태 모니터링

## 설치 및 실행

### 1. 패키지 설치
```bash
cd /workspaces/rainbow-robot-connect/RainbowRobot_ConnectTest/src/rb_web
pip install -r requirements.txt
```

### 2. 웹 서버 실행
```bash
# 방법 1: 직접 실행
python3 rb_web_test.py

# 방법 2: 스크립트 사용
bash run_web_server.sh
```

### 3. 웹 브라우저 접속
```
http://localhost:5000
```

## API 엔드포인트

### GET/POST `/api/status`
현재 시스템 상태 조회
```json
{
  "busy": false,
  "selected_file": "/path/to/motion.yaml",
  "last_status": "준비됨",
  "servo_params": {
    "t1": 0.05,
    "t2": 0.05,
    "gain": 0.1,
    "alpha": 0.03
  },
  "home_pose": [0.0, -45.0, 137.0, 0.0, -90.0, 0.0]
}
```

### GET `/api/motions-list`
사용 가능한 모션 파일 목록
```json
{
  "files": [
    {
      "name": "home_motion.yaml",
      "path": "/workspace/motions/home_motion.yaml",
      "relative": "home_motion.yaml",
      "size": 1024,
      "modified": "2025-01-22T10:30:00"
    }
  ],
  "directory": "/workspace/motions"
}
```

### POST `/api/load-motion`
모션 파일 선택
```json
// Request
{
  "filepath": "/workspace/motions/motion.yaml"
}

// Response
{
  "success": true,
  "selected_file": "/workspace/motions/motion.yaml",
  "status": "모션 파일 로드됨: motion.yaml"
}
```

### POST `/api/run-home`
Home 이동 실행
```json
{
  "success": true,
  "busy": true,
  "status": "홈 이동: [0.0, -45.0, 137.0, 0.0, -90.0, 0.0]"
}
```

### POST `/api/run-motion`
선택한 모션 파일 실행
```json
{
  "success": true,
  "busy": true,
  "status": "모션 실행 완료"
}
```

### GET/POST `/api/servo-params`
ServoJ 파라미터 조회/수정

**GET 응답:**
```json
{
  "t1": 0.05,
  "t2": 0.05,
  "gain": 0.1,
  "alpha": 0.03
}
```

**POST 요청:**
```json
{
  "t1": 0.08,
  "t2": 0.06,
  "gain": 0.15,
  "alpha": 0.05
}
```

### GET/POST `/api/home-pose`
Home 위치 정보 조회/수정

**GET 응답:**
```json
{
  "home_pose": [0.0, -45.0, 137.0, 0.0, -90.0, 0.0],
  "home_speed": 20.0,
  "home_accel": 20.0
}
```

**POST 요청:**
```json
{
  "home_pose": [10.0, -40.0, 140.0, 5.0, -85.0, 5.0],
  "home_speed": 25.0,
  "home_accel": 25.0
}
```

## 폴더 구조

```
rb_web/
├── rb_web_test.py           # 메인 Flask 애플리케이션
├── requirements.txt         # Python 패키지 의존성
├── run_web_server.sh        # 실행 스크립트
├── templates/
│   └── index.html          # 웹 UI 템플릿
└── static/
    ├── style.css           # 스타일시트
    └── script.js           # 클라이언트 JavaScript
```

## motion_gui_runner.py에서 포팅된 코드

### 1. Home 이동 (run_home)
```python
# 원본: motion_gui_runner.py의 RunnerNode.run_home()
def run_home(self) -> bool:
    # cobot.MoveJ(j0, j1, j2, j3, j4, j5, speed, accel) 호출
```

### 2. 모션 파일 관리
```python
# 원본: motion_gui_runner.py의 _default_motions_dir()
def _default_motions_dir() -> str:
    ws = Path(_guess_workspace_root())
    return str((ws / "motions").resolve())
```

### 3. ServoJ 파라미터
```python
# 원본: motion_gui_runner.py의 set_servo_overrides()
def set_servo_params(self, t1=None, t2=None, gain=None, alpha=None):
    # MotionExecutor에 파라미터 전달
```

### 4. 모션 실행
```python
# 원본: motion_gui_runner.py의 run_motion_file()
def run_motion_file(self, filepath: str = None) -> bool:
    # MotionExecutor.load_motion_from_file() 호출
```

## 파라미터 설명

### Home 파라미터
- **home_pose_arr**: [j0, j1, j2, j3, j4, j5] - 홈 위치 조인트 각도
- **home_speed**: 이동 속도 (deg/s)
- **home_accel**: 가속도 (deg/s²)

### ServoJ 파라미터
- **t1**: Time constant 1 (시간 상수 1)
- **t2**: Time constant 2 (시간 상수 2)
- **gain**: 제어 이득 (피드백 게인)
- **alpha**: 필터 계수 (저역 필터)

## 웹 UI 사용 방법

### 1. 상태 확인
- 메인 페이지 상단의 "시스템 상태" 섹션에서 현재 상태 확인
- 실행 중 여부, 선택된 파일 등을 표시

### 2. 모션 파일 선택
- "모션 파일 선택" 섹션에서 파일 목록 조회
- 원하는 파일 옆의 "선택" 버튼 클릭
- 또는 파일 자체를 클릭

### 3. Home 이동
- "Home 이동" 섹션에서:
  - Home 위치 수정 (선택사항)
  - 이동 속도/가속도 조절 (선택사항)
  - "Home으로 이동" 버튼 클릭

### 4. ServoJ 파라미터 조정
- "ServoJ 파라미터" 섹션에서:
  - t1, t2, gain, alpha 값 수정
  - "ServoJ 파라미터 업데이트" 버튼 클릭
  - 현재 값이 하단에 표시됨

### 5. 모션 실행
- "모션 실행" 섹션에서:
  - 선택된 파일 확인
  - "모션 실행" 버튼 클릭
  - 로그에서 진행 상황 확인

## 주의사항

1. **ROS2 필수**: 웹 서버가 ROS2 환경에서 실행되어야 합니다
2. **워크스페이스 경로**: `SMART_WS_DIR` 환경변수 또는 자동 추론으로 워크스페이스 위치 결정
3. **motions 디렉터리**: `<workspace>/motions` 폴더에 YAML/JSON 파일 저장
4. **MotionExecutor**: inline 모드에서만 지원됨 (process 모드 미지원)

## 문제 해결

### 웹 페이지가 로드되지 않음
- Flask 서버가 실행 중인지 확인
- `http://localhost:5000`이 접근 가능한지 확인
- 방화벽 설정 확인

### 모션 파일이 나타나지 않음
- `~/motions` 디렉터리 존재 확인
- YAML/JSON 파일 확장자 확인
- 콘솔 로그에서 디렉터리 경로 확인

### Home 이동이 작동하지 않음
- `cobot` 모듈이 올바르게 import되었는지 확인
- 로봇이 준비 상태인지 확인
- 콘솔 로그에서 에러 메시지 확인

## 라이선스

기존 `motion_gui_runner.py` 기반 개발

## 참고 자료

- 원본: `/src/rb_test/motion_gui_runner.py`
- Cobot API: `/src/rb_test/cobot.py`
- MotionExecutor: `/src/rb_test/motion_executor.py`
