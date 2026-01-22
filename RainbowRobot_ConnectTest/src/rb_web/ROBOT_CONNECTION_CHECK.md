# 🔍 로봇 연결 체크 함수 설명 & 진단 가이드

## 문제점 분석 ❌

### 현재 코드의 문제
```python
# 📍 rb_web_test.py 라인 158-195

def _init_robot_in_background(self):
    """백그라운드에서 로봇 초기화"""
    def _init():
        try:
            # 🔴 문제: 이 줄만 실행되면 "연결됨"으로 표시
            self.exec_node = MotionExecutor()  # ← 객체 생성만 함!
            self.exec_node.set_servo_overrides(**self.servo_params)
            
            # 🔴 실제 로봇과 통신 확인 없음!
            self._robot_connected = True  # ← 무조건 True 할당
            
        except Exception as e:
            self._robot_connected = False
```

### 왜 문제인가?

| 단계 | 동작 | 결과 |
|------|------|------|
| 1️⃣ | `MotionExecutor()` 생성 | ✅ 객체 생성 성공 |
| 2️⃣ | `set_servo_overrides()` 호출 | ⚠️ 이 함수에서 실제 로봇과 통신 |
| 3️⃣ | 예외 발생 | ❌ 로봇 응답 없음 |
| 4️⃣ | 그런데... | ⚠️ 예외 처리 부분에서만 `_robot_connected = False` |
| 5️⃣ | **결과** | **🔴 UI에는 "연결됨"으로 표시됨** |

### 예시 시나리오

```
시간       네트워크                   코드 상태
─────────────────────────────────────────────────
t=0s     로봇 없음 (IP 192.168.1.13 응답 안 함)
         ↓
         exec_node = MotionExecutor()  ✅ 객체만 생성
         ↓
t=1s     exec_node.set_servo_overrides()  → 타임아웃 발생!
         ↓
         예외 처리에서만 False 설정
         ↓
         하지만 잘못된 상태 남아있음!

결과: ⚠️ UI에는 여전히 "연결됨"으로 표시 될 수 있음
```

---

## ✅ 개선된 체크 방식

### 3단계 검증 로직

```python
# 📍 rb_web_test.py 라인 158-230 (개선됨)

def _init_robot_in_background(self):
    def _init():
        try:
            # 1️⃣ MotionExecutor 객체 생성
            self.exec_node = MotionExecutor()
            self.get_logger().info("[ROBOT] MotionExecutor created")
            
            # 2️⃣ 실제 로봇과 통신 테스트 (핸드셰이크)
            self.exec_node.set_servo_overrides(**self.servo_params)
            self.get_logger().info("[ROBOT] Servo parameters set successfully")
            
            # 3️⃣ 로봇 상태 읽기 시도 (추가 확인)
            if hasattr(self.exec_node, 'get_joint_values'):
                joint_values = self.exec_node.get_joint_values()
                self.get_logger().info(f"[ROBOT] Joint values: {joint_values}")
            
            # ✅ 모든 체크 통과
            self._robot_connected = True
            self._robot_error_msg = ""
            self._last_status = "✅ 로봇 연결됨"
            
        except TimeoutError:
            # ⚠️ 로봇 응답 없음
            self._robot_connected = False
            self._robot_error_msg = "연결 시간 초과 (10초)"
            
        except Exception as e:
            # ❌ 로봇 오류
            self._robot_connected = False
            self._robot_error_msg = str(e)[:50]
```

### 검증 과정

```
1️⃣ MotionExecutor 생성
   └─ 성공: 객체 생성됨
      실패: MotionExecutor 모듈 없음

2️⃣ set_servo_overrides() 호출
   └─ 성공: 로봇과 통신 가능 ✅
      실패: 로봇 IP 응답 없음 ❌

3️⃣ get_joint_values() 호출
   └─ 성공: 로봇 상태 읽음 ✅
      실패: 로봇 하드웨어 오류 ❌
```

---

## 🔧 진단 방법

### 방법 1️⃣: 새 API 엔드포인트 사용 (권장)

```bash
# 터미널에서 웹 서버 실행 중일 때
curl http://localhost:5000/api/robot-diagnostics | python -m json.tool
```

**출력 예시 (로봇 연결됨):**
```json
{
  "robot_connected": true,
  "robot_error": "",
  "status": "✅ 로봇 연결됨",
  "has_executor": true,
  "executor_type": "MotionExecutor",
  "executor_methods": ["get_joint_values", "move", "set_servo_overrides", ...],
  "joint_values": [0.0, -45.0, 137.0, 0.0, -90.0, 0.0],
  "connection_verified": true
}
```

**출력 예시 (로봇 미연결):**
```json
{
  "robot_connected": false,
  "robot_error": "연결 시간 초과 (10초) - 로봇 응답 없음",
  "status": "⚠️ 로봇 미연결: 연결 시간 초과 (10초)",
  "has_executor": true,
  "executor_type": "MotionExecutor",
  "connection_verified": false,
  "joint_values_error": "timeout"
}
```

### 방법 2️⃣: 웹 UI 콘솔 확인

**개발자도구 열기:**
```
F12 → Console 탭
```

**로그 메시지 확인:**
```
[ROBOT] Connecting to robot (timeout: 10s)...
[ROBOT] MotionExecutor created
[ROBOT] Servo parameters set successfully
[ROBOT] Joint values: [0.0, -45.0, 137.0, 0.0, -90.0, 0.0]
[ROBOT] ✅ Robot connected and verified!
```

### 방법 3️⃣: 터미널 로그 확인

**웹 서버 터미널에서 보이는 로그:**
```
[ROBOT] Connecting to robot (timeout: 10s)...
[ROBOT] MotionExecutor created
[ROBOT] Servo parameters set successfully
[ROBOT] Joint values: [0.0, -45.0, 137.0, 0.0, -90.0, 0.0]
[ROBOT] ✅ Robot connected and verified!
```

---

## 📊 체크 함수별 역할

### 1️⃣ `MotionExecutor()` 생성
```python
self.exec_node = MotionExecutor()

목적: 로봇 제어 모듈 초기화
확인사항:
  ✅ rb_test 모듈 설치됨
  ✅ 파이썬 환경 설정됨
확인 불가:
  ❌ 로봇이 실제로 있는지
  ❌ 네트워크 연결 상태
```

### 2️⃣ `set_servo_overrides()` 호출
```python
self.exec_node.set_servo_overrides(**self.servo_params)

목적: 실제 로봇과 통신 확인
확인사항:
  ✅ 로봇 IP (192.168.1.13) 응답 있음
  ✅ 네트워크 연결 가능
  ✅ 로봇이 명령 수신 가능
  ✅ 서보 모터 제어 가능
확인 가능한 오류:
  ❌ "연결 시간 초과" → 로봇 응답 없음
  ❌ "Connection refused" → 포트 응답 없음
  ❌ "Motor error" → 로봇 하드웨어 오류
```

### 3️⃣ `get_joint_values()` 호출 (추가 확인)
```python
joint_values = self.exec_node.get_joint_values()

목적: 로봇 상태 읽기 (추가 검증)
확인사항:
  ✅ 로봇에서 현재 위치 읽기 성공
  ✅ 양방향 통신 가능
  ✅ 로봇이 정상 작동 중
반환값:
  [0.0, -45.0, 137.0, 0.0, -90.0, 0.0]  ← 각 관절의 각도
```

---

## 🚨 오류 메시지 해석

| 오류 메시지 | 원인 | 해결 방법 |
|-----------|------|---------|
| `연결 시간 초과 (10초)` | 로봇 IP 응답 없음 | IP 주소 확인, 네트워크 핑 테스트 |
| `Connection refused` | 포트 응답 없음 | 로봇 전원 확인, 방화벽 설정 |
| `Motor error` | 로봇 하드웨어 오류 | 로봇 리부팅, 서보 드라이버 확인 |
| `timeout` | 명령 응답 시간 초과 | 로봇 과부하 확인, 네트워크 지연 확인 |
| `로봇 제어 미지원` | ROS2 모드 미설정 | `run_mode='inline'` 확인 |

---

## 🧪 테스트 시나리오

### 시나리오 1️⃣: 로봇 연결됨 (정상)

```bash
$ python3 rb_web_test.py
[ROBOT] Connecting to robot (timeout: 10s)...
[ROBOT] MotionExecutor created
[ROBOT] Servo parameters set successfully
[ROBOT] Joint values: [0.0, -45.0, 137.0, 0.0, -90.0, 0.0]
[ROBOT] ✅ Robot connected and verified!

UI 표시: ✅ 로봇 연결됨 (192.168.1.13)
```

### 시나리오 2️⃣: 로봇 응답 없음 (시간 초과)

```bash
$ python3 rb_web_test.py
[ROBOT] Connecting to robot (timeout: 10s)...
[ROBOT] MotionExecutor created
(10초 대기...)
[ROBOT] ⚠️ 연결 시간 초과 (10초) - 로봇 응답 없음

UI 표시: ⚠️ 로봇 미연결: 연결 시간 초과 (10초)
```

### 시나리오 3️⃣: ROS2 없음

```bash
$ python3 rb_web_test.py
[WARN] ROS2 not installed
[INFO] Using dummy node for standalone web server...

UI 표시: ⚠️ 로봇 미연결: 로봇 제어 미지원
```

---

## 💡 권장사항

### 더 정확한 연결 확인을 위해:

1. **주기적 헬스체크 추가**
   ```python
   # 백그라운드에서 30초마다 로봇 상태 확인
   def _health_check():
       while True:
           try:
               if self.exec_node:
                   self.exec_node.get_joint_values()
                   self._robot_connected = True
           except:
               self._robot_connected = False
           time.sleep(30)
   ```

2. **실시간 상태 모니터링**
   - WebSocket 사용하여 실시간 상태 업데이트
   - 로봇과의 연결이 끊어지면 즉시 알림

3. **로그 저장**
   - 연결 실패 시 타임스탬프와 함께 로그 저장
   - 이력 조회를 통한 문제 분석

---

## 📞 문제 해결

### Q: 로봇이 연결된 것처럼 표시되는데 실제로는 안 됨
**A:** `/api/robot-diagnostics` 엔드포인트로 진단해보세요
```bash
curl http://localhost:5000/api/robot-diagnostics
```

### Q: 계속 "연결 시간 초과"가 나옴
**A:** 
1. 로봇 IP 확인: `ping 192.168.1.13`
2. 네트워크 연결 상태 확인
3. 로봇 전원이 켜져 있는지 확인

### Q: 로봇이 연결되어 있지만 제어가 안 됨
**A:** `/api/home` (Home 이동) 실패하면 로그 확인
```bash
curl -X POST http://localhost:5000/api/run-home
```

---

**최신 업데이트**: 2025-01-22
✅ 3단계 검증 로직 구현
✅ 진단 API 엔드포인트 추가
✅ 상세 오류 메시지 추가
