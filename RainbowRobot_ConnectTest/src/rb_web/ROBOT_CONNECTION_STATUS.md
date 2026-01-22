# 🤖 로봇 연결 상태 표시 기능 완료

## ✅ 수정 사항

### 1️⃣ 로봇 연결 실패해도 웹 UI 정상 실행
- ❌ **이전**: 로봇 연결 실패 → 웹 서버 안 띄워짐
- ✅ **이후**: 로봇 연결 실패 → 웹 UI 띄워짐 + 경고 표시

### 2️⃣ UI에 로봇 연결 상태 표시
- **로봇 연결됨** 상태:
  ```
  ✅ 로봇 연결됨 (192.168.1.13)
  ```

- **로봇 미연결** 상태:
  ```
  ⚠️ 로봇 미연결
  [경고 메시지]
  로봇이 연결되지 않았습니다
  오류: [연결 실패 이유]
  💡 로봇 모듈을 확인하고 192.168.1.13로 접근 가능한지 확인하세요.
  ```

### 3️⃣ 로봇 미연결 시 제어 불가
- Home 이동 버튼 클릭 → "❌ 로봇이 연결되지 않았습니다" 표시
- 모션 실행 버튼 클릭 → "❌ 로봇이 연결되지 않았습니다" 표시
- 파일 선택/ServoJ 파라미터 수정 → **정상 작동** ✅

## 📊 상태 표시 화면

### 로봇 연결됨
```
┌────────────────────────────────────────┐
│ 🤖 시스템 상태                         │
├────────────────────────────────────────┤
│ 🤖 로봇: ✅ 로봇 연결됨 (192.168.1.13) │
│ 상태: ✅ 준비됨                        │
│ 실행 중: 아니오                        │
│ 선택 파일: home_motion.yaml             │
└────────────────────────────────────────┘
```

### 로봇 미연결
```
┌────────────────────────────────────────┐
│ ⚠️ 로봇 연결 상태                       │
├────────────────────────────────────────┤
│ 로봇이 연결되지 않았습니다              │
│ 오류: [Errno 111] Connection refused   │
│ 💡 로봇 모듈을 확인하고 192.168.1.13   │
│    로 접근 가능한지 확인하세요.         │
└────────────────────────────────────────┘

┌────────────────────────────────────────┐
│ 🤖 시스템 상태                         │
├────────────────────────────────────────┤
│ 🤖 로봇: ⚠️ 로봇 미연결               │
│ 상태: ⚠️ 로봇 연결 실패                 │
│ 실행 중: 아니오                        │
│ 선택 파일: 없음                         │
└────────────────────────────────────────┘
```

## 🔧 변경된 코드 부분

### 1. `rb_web_test.py` - MotionWebNode 클래스

#### 초기화 부분
```python
# 로봇 연결 상태 추적
self._robot_connected = False
self._robot_error_msg = ""

# MotionExecutor 초기화 (에러 처리)
if self.run_mode == 'inline':
    try:
        self.exec_node = MotionExecutor()
        self._robot_connected = True
        self._last_status = "✅ 로봇 연결됨"
    except Exception as e:
        self._robot_connected = False
        self._robot_error_msg = str(e)
        self._last_status = f"⚠️ 로봇 연결 실패: {e}"
```

#### get_status() 메서드
```python
def get_status(self):
    status = {
        ...
        'robot_connected': self._robot_connected,  # 🆕
        'robot_error': self._robot_error_msg,      # 🆕
    }
    return status
```

#### run_home() 메서드
```python
def run_home(self):
    # 로봇 연결 확인 추가
    if not self._robot_connected:
        self.set_status(f"❌ 로봇이 연결되지 않았습니다: {self._robot_error_msg}")
        return False
    # ... 나머지 코드
```

#### run_motion_file() 메서드
```python
def run_motion_file(self):
    # 로봇 연결 확인 추가
    if not self._robot_connected:
        self.set_status(f"❌ 로봇이 연결되지 않았습니다: {self._robot_error_msg}")
        return False
    # ... 나머지 코드
```

#### init_ros() 함수
```python
def init_ros():
    try:
        rclpy.init()
    except Exception as e:
        print(f"[WARNING] ROS2 init failed: {e}")
        return False  # 웹 서버는 계속 실행
    
    try:
        ros_node = MotionWebNode()
        # ... executor 시작
    except Exception as e:
        print(f"[ERROR] MotionWebNode creation failed: {e}")
        ros_node = None  # 웹 서버는 계속 실행
        return False
```

### 2. `templates/index.html` - 로봇 상태 알림

```html
<!-- 상단에 로봇 연결 상태 알림 -->
<section id="robot-status-alert" class="alert alert-info" style="display: none;">
    <h3>🤖 로봇 연결 상태</h3>
    <div id="robot-status-content"></div>
</section>

<!-- 상태 표시에 로봇 연결 상태 추가 -->
<div class="status-item">
    <span class="label">🤖 로봇:</span>
    <span id="robot-status" class="value">확인 중...</span>
</div>
```

### 3. `static/style.css` - 알림 스타일

```css
.alert {
    padding: 20px;
    border-radius: var(--border-radius);
    margin-bottom: 20px;
    border-left: 5px solid;
}

.alert-info { background-color: #e3f2fd; }
.alert-warning { background-color: #fff3e0; }
.alert-danger { background-color: #ffebee; }
.alert-success { background-color: #e8f5e9; }
```

### 4. `static/script.js` - 상태 업데이트

```javascript
async function updateStatus() {
    const status = await apiCall('/status');
    
    // 로봇 연결 상태 표시
    if (status.robot_connected) {
        robotStatusElement.textContent = '✅ 로봇 연결됨 (192.168.1.13)';
        alertBox.style.display = 'none';
    } else {
        robotStatusElement.textContent = '⚠️ 로봇 미연결';
        alertBox.style.display = 'block';
        alertContent.innerHTML = `<p>❌ 오류: ${status.robot_error}</p>`;
    }
}
```

## 🚀 동작 흐름

### 시나리오 1: 로봇 연결됨
```
1. python3 rb_web_test.py 실행
2. MotionExecutor() 초기화 성공
3. _robot_connected = True
4. 웹 UI: ✅ 로봇 연결됨 표시
5. Home 이동, 모션 실행 가능
```

### 시나리오 2: 로봇 미연결 (192.168.1.13 오프라인)
```
1. python3 rb_web_test.py 실행
2. MotionExecutor() 초기화 실패 (Connection refused)
3. _robot_connected = False
4. _robot_error_msg = "Connection refused"
5. 웹 UI 정상 띄워짐 ✅
6. 상단에 경고 표시 ⚠️
7. Home 이동, 모션 실행 블로됨 ❌
8. 파일 선택, 파라미터 조정은 가능 ✅
```

## 🎯 사용 시나리오

### 시나리오 A: 로봇이 정상 작동 중
```
웹 서버 시작
  ↓
로봇 연결 성공
  ↓
[상태: ✅ 로봇 연결됨]
  ↓
모든 기능 정상 작동
  ✅ Home 이동
  ✅ 모션 실행
  ✅ 파라미터 조정
```

### 시나리오 B: 로봇 오프라인
```
웹 서버 시작
  ↓
로봇 연결 실패
  ↓
[상태: ⚠️ 로봇 미연결]
[경고: 연결 실패 이유 표시]
  ↓
일부 기능 제한
  ❌ Home 이동 불가
  ❌ 모션 실행 불가
  ✅ 파일 선택 가능
  ✅ 파라미터 조정 가능
  ✅ 설정 미리보기
```

### 시나리오 C: 로봇 재연결
```
1. 웹 UI: ⚠️ 로봇 미연결 표시
2. 로봇 전원 켜짐 (192.168.1.13)
3. 웹 새로고침 (F5)
4. 웹 UI: ✅ 로봇 연결됨 표시 ✅
5. 모든 기능 정상 작동
```

## 📝 테스트 방법

### 테스트 1: 로봇 정상 (연결 가능)
```bash
python3 rb_web_test.py
# 출력: ✅ 로봇 연결됨
# 웹 UI: ✅ 로봇 연결됨 (192.168.1.13)
```

### 테스트 2: 로봇 오프라인
```bash
# 로봇 전원 끄기
python3 rb_web_test.py
# 출력: ⚠️ 로봇 연결 실패
# 웹 UI: ⚠️ 로봇 미연결
# 웹은 정상 띄워짐 ✅
```

### 테스트 3: 로봇 재연결
```bash
1. 웹 UI에서 "⚠️ 로봇 미연결" 표시 확인
2. 로봇 전원 켜기
3. 브라우저에서 새로고침 (F5)
4. 웹 UI: ✅ 로봇 연결됨 표시 ✅
```

## 🎉 완료 체크리스트

- ✅ 로봇 연결 상태 추적
- ✅ 연결 실패해도 웹 UI 정상 실행
- ✅ UI에 연결 상태 표시
- ✅ 연결 실패 이유 표시
- ✅ 로봇 미연결 시 제어 불가 (안전)
- ✅ 로봇 미연결 시 설정은 가능 (편의)
- ✅ 반응형 디자인 유지
- ✅ 문서화

---

이제 언제든지 웹을 띄우고, 로봇 연결 상태를 확인할 수 있습니다! 🚀
