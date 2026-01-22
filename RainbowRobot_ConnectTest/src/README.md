# RB Test Package

ROS2 package for robot control with Modbus communication and motion execution GUI.

## 구성 요소

### 핵심 노드
1. **rb_modbus_bridge**: Modbus TCP를 통해 디지털 입력 신호를 모니터링하고 `/reflect_sensor` 토픽으로 발행
2. **motion_gui_runner**: GUI 인터페이스로 모션 파일을 선택하고 센서 신호에 따라 자동 실행

### 파일 구조
```
rb_test/
├── rb_test/                    # Python 모듈
│   ├── rb_modbus_bridge.py    # Modbus 통신 노드
│   ├── motion_gui_runner.py   # 모션 실행 GUI
│   └── ...                    # 기타 모듈
├── launch/                     # Launch 파일
│   ├── rb_modbus_bridge.launch.py
│   ├── motion_gui_runner.launch.py
│   └── rb_test_all.launch.py  # 통합 런처
├── package.xml                 # 패키지 설정
├── setup.py                    # Python 설정
└── CMakeLists.txt             # CMake 설정
```

## 빠른 시작

### 1. 의존성 설치
```bash
pip3 install pymodbus PyQt5 multipledispatch transforms3d numpy
```

### 2. 빌드
```bash
cd /root/smart_ws
./build_rb_test.sh
```

### 3. 실행
```bash
# 모든 노드 실행 (기본값 사용)
./run_rb_test.sh

# 커스텀 설정으로 실행
./run_rb_test.sh -i 192.168.1.10 -d 12 -t falling_delay

# 대화형 관리자 사용
./rb_test_manager.sh
```

## 셸 스크립트 설명

### build_rb_test.sh
- 패키지 빌드 자동화
- 의존성 확인 및 설치
- 빌드 상태 리포트

### run_rb_test.sh
- 다양한 실행 모드 지원 (all, modbus, gui, manual)
- 명령줄 매개변수 지원
- 설정 가능한 옵션:
  - `-m, --mode`: 실행 모드 선택
  - `-i, --ip`: 로봇 IP 주소
  - `-d, --di`: 디지털 입력 주소
  - `-t, --trigger`: 트리거 모드

### rb_test_manager.sh
- 대화형 관리 인터페이스
- 빌드, 실행, 모니터링, 디버깅 통합
- 시스템 체크 및 의존성 관리
- Quick Start (빌드 + 실행) 옵션

## Launch 파일 사용법

### 개별 노드 실행
```bash
# Modbus Bridge만 실행
ros2 launch rb_test rb_modbus_bridge.launch.py robot_ip:=192.168.1.13 di_address:=12

# Motion GUI Runner만 실행
ros2 launch rb_test motion_gui_runner.launch.py trigger_mode:=falling_delay
```

### 통합 실행
```bash
# 두 노드 모두 실행
ros2 launch rb_test rb_test_all.launch.py robot_ip:=192.168.1.13 di_address:=12 trigger_mode:=falling_delay
```

## 주요 매개변수

### rb_modbus_bridge
- `robot_ip`: 로봇 컨트롤러 IP (기본값: 192.168.1.13)
- `port`: Modbus TCP 포트 (기본값: 502)
- `di_address`: 디지털 입력 주소 (기본값: 12)
- `poll_ms`: 폴링 간격 (기본값: 50ms)
- `function`: Modbus 함수 타입 ('di' 또는 'coil')
- `slave_id`: Modbus 슬레이브 ID (기본값: 1)

### motion_gui_runner
- `reflect_topic`: 센서 신호 토픽 (기본값: /reflect_sensor)
- `trigger_mode`: 트리거 모드 ('falling_delay' 또는 'rising')
- `post_clear_delay_ms`: 신호 1→0 후 실행 지연 (기본값: 5000ms)
- `cooldown_ms`: 쿨다운 시간 (기본값: 2000ms)
- `home_yaml`: 홈 모션 YAML 파일 경로

## 문제 해결

### 빌드 실패 시
```bash
# 클린 빌드
./rb_test_manager.sh
# 옵션 7 선택 (Clean Build)
# 옵션 1 선택 (Build Package)
```

### 노드 상태 확인
```bash
# 실행 중인 노드 확인
ros2 node list

# 토픽 확인
ros2 topic list

# 토픽 모니터링
ros2 topic echo /reflect_sensor
```

## 개발 팁

1. **디버그 모드**: Launch 파일에서 `output='screen'` 설정으로 콘솔 출력 확인
2. **매개변수 덮어쓰기**: Launch 시 명령줄에서 매개변수 덮어쓰기 가능
3. **시스템 체크**: `./rb_test_manager.sh`의 옵션 9로 시스템 상태 확인

## 라이센스
Apache License 2.0