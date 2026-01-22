# Rainbow Robot Web Control - 빠른 시작 가이드 (개선됨)

## 주요 개선사항 ✨

### 문제 해결 ✅
- **웹 서버 블로킹 문제 해결**: 로봇 연결 시간 초과로 웹이 띄워지지 않던 문제 **완전 해결**
- **비동기 로봇 연결**: 로봇 연결을 백그라운드 스레드에서 비동기 처리 (타임아웃 10초)
- **ROS2 선택사항**: ROS2 없어도 웹 서버 실행 가능

### 동작 방식

```
시작 → 웹 서버 즉시 띄우기 (메인 스레드)
        ↓
        백그라운드 스레드에서 로봇 연결 시도 (타임아웃 10초)
        ↓
        로봇 연결 상태는 웹 UI에서 실시간 표시
```

## 설치 및 실행

### 1단계: 필요한 패키지 설치

```bash
cd ~/rainbow-robot-connect/RainbowRobot_ConnectTest/src/rb_web
pip install -r requirements.txt
```

### 2단계: 웹 서버 시작

```bash
python3 rb_web_test.py
```

**출력 예시:**
```
Rainbow Robot Web Control

[INIT] Starting Rainbow Robot Web Control...
[WARN] ROS2 not installed
[INFO] Using dummy node for standalone web server...

[WEB] Starting Flask web server...
[WEB] Open browser at: http://localhost:5000
```

### 3단계: 웹 브라우저에서 접속

```
http://localhost:5000
```

## 동작 플로우

### 로봇 연결 상태에 따른 UI 표시

#### 상황 1: 로봇 연결 중 (처음 5초)
```
⏳ 로봇 연결 중... (타임아웃: 10초)
- 웹은 정상 작동 ✅
- 제어 버튼은 비활성화 (회색)
```

#### 상황 2: 로봇 연결 성공 ✅
```
✅ 로봇 연결됨 (192.168.1.13)
- 모든 제어 버튼 활성화 🟢
- Home 이동 가능
- Motion 파일 실행 가능
- ServoJ 파라미터 조정 가능
```

#### 상황 3: 로봇 미연결 ⚠️
```
⚠️ 로봇 미연결: 연결 시간 초과 (10초)
- 웹은 정상 작동 ✅
- 제어 버튼은 비활성화 (회색)
- 모션 파일 선택은 가능 (나중에 로봇 연결되면 실행)
```

## 각 페이지별 기능

### 1. 상태 패널 (Status Panel)
```
현재 상태: [상태 메시지]
로봇 연결: ✅ 로봇 연결됨 / ⚠️ 로봇 미연결
```

### 2. Home 이동
```
[🏠 Home 이동 버튼]
- 현재 위치: [0.0, -45.00, 137.00, 0.00, -90.00, 0.0]
- Speed: 20.0
- Accel: 20.0
```

### 3. 모션 파일 관리
```
[📂 디렉터리] ~/motions
[🔍 파일 목록]
- 발보일1.yaml
- 배선트레이.yaml
- 하우징M.yaml
... 등
```

### 4. ServoJ 파라미터
```
t1:   [___] 
t2:   [___]
gain: [___]
alpha:[___]
[💾 저장 버튼]
```

### 5. 모션 실행
```
[📁 선택한 파일: 발보일1.yaml]
[▶️ 모션 실행 버튼]
```

### 6. 실행 로그
```
2025-01-22 10:30:45 - ✅ 홈 위치 이동 완료
2025-01-22 10:31:20 - ⚠️ 로봇 미연결
```

## 요구사항

### 최소 요구사항 (웹만 실행)
- Python 3.8+
- Flask 2.3.2+
- flask-cors
- pyyaml

### 로봇 제어 (완전 기능)
- Python 3.8+
- ROS2 Humble
- rb_test 모듈 (rainbow-robot-connect)
- 로봇 하드웨어 (192.168.1.13)

## 문제 해결

### Q: 웹이 안 띄워집니다
**A:** 포트 5000이 사용 중인지 확인
```bash
lsof -i :5000
# 있으면 kill -9 [PID]
```

### Q: 로봇이 연결되지 않습니다
**A:** 정상입니다! 로봇과 PC 네트워크 연결을 확인하세요
- 로봇 IP: 192.168.1.13
- 웹은 계속 작동합니다 ✅

### Q: 홈 이동이 안 됩니다
**A:** 로봇 연결 상태 확인
- UI에서 "⚠️ 로봇 미연결" 표시되면 로봇과 연결 필요
- ROS2 설치 필요

## 파일 구조

```
rb_web/
├── rb_web_test.py          ⭐ 메인 웹 서버 (개선 버전)
├── rb_web_test_old.py      (백업)
├── requirements.txt        ✅ 필요한 패키지
├── templates/
│   └── index.html          웹 UI 템플릿
├── static/
│   ├── style.css           스타일시트
│   └── script.js           클라이언트 로직
├── README.md
└── QUICK_START_Ubuntu22.md (Ubuntu 22.04 설정)
```

## API 엔드포인트

| 메서드 | URL | 설명 |
|--------|-----|------|
| GET | `/` | 웹 UI 페이지 |
| GET | `/api/status` | 현재 상태 조회 |
| GET | `/api/motions-list` | 모션 파일 목록 |
| POST | `/api/load-motion` | 모션 파일 선택 |
| POST | `/api/run-home` | Home 이동 실행 |
| POST | `/api/run-motion` | 선택한 모션 실행 |
| GET/POST | `/api/servo-params` | ServoJ 파라미터 조회/수정 |
| GET/POST | `/api/home-pose` | Home 위치 조회/수정 |

## 로그 메시지 해석

```
[INIT] - 초기화 단계
[ROS2] - ROS2 관련
[ROBOT] - 로봇 연결 관련
[WEB] - 웹 서버 관련
[ERROR] - 오류 발생
[WARN] - 경고
```

## 다음 단계

1. **로봇 연결 필요**: [ROS2_INSTALL_GUIDE.md](ROS2_INSTALL_GUIDE.md) 참조
2. **배포**: 프로덕션 환경에서는 Gunicorn 사용 권장
3. **보안**: HTTPS, 인증 추가 고려

---

**최신 업데이트**: 2025-01-22
- ✅ 비동기 로봇 연결 구현
- ✅ ROS2 선택사항화
- ✅ 웹 서버 즉시 시작
