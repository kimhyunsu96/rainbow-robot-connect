#!/bin/bash
# Rainbow Robot - ROS2 Humble 설치 스크립트 (Ubuntu 22.04.5 LTS)
# 실행: bash install_ros2.sh

set -e

echo "=========================================="
echo "🤖 ROS2 Humble 설치 스크립트 (Ubuntu 22.04)"
echo "=========================================="
echo ""

# 1. Locale 설정
echo "📍 Step 1: Locale 설정..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 필수 패키지 설치
echo "📍 Step 2: 필수 패키지 설치..."
sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    python3-pip

# 3. ROS2 GPG 키 추가
echo "📍 Step 3: ROS2 GPG 키 추가..."
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. ROS2 Repository 추가
echo "📍 Step 4: ROS2 Repository 추가..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 저장소 업데이트
echo "📍 Step 5: 패키지 저장소 업데이트..."
sudo apt update

# 6. ROS2 Humble Desktop 설치
echo "📍 Step 6: ROS2 Humble Desktop 설치..."
echo "이 단계는 시간이 걸릴 수 있습니다 (5-15분)..."
sudo apt install -y ros-humble-desktop

# 7. Build Tools 설치
echo "📍 Step 7: Build Tools 설치..."
sudo apt install -y ros-humble-ros-core
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep2

# 8. Shell 자동완성 설치 (선택사항)
echo "📍 Step 8: Shell 자동완성 설정..."
sudo apt install -y python3-argcomplete
mkdir -p ~/.bash_completion.d/
sudo cp /etc/bash_completion.d/colcon /etc/bash_completion.d/colcon 2>/dev/null || true

# 9. .bashrc에 ROS2 설정 추가
echo "📍 Step 9: .bashrc에 ROS2 설정 추가..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble 설정" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
fi

# 10. 현재 세션에 ROS2 설정 적용
source /opt/ros/humble/setup.bash

echo ""
echo "=========================================="
echo "✅ ROS2 Humble 설치 완료!"
echo "=========================================="
echo ""
echo "📌 다음 단계:"
echo "1. 새 터미널을 열거나 다음 명령 실행:"
echo "   source ~/.bashrc"
echo ""
echo "2. 설치 확인:"
echo "   ros2 --version"
echo ""
echo "3. 데모 실행 (선택사항):"
echo "   ros2 run demo_nodes_cpp talker  (터미널 1)"
echo "   ros2 run demo_nodes_cpp listener (터미널 2)"
echo ""
echo "4. Rainbow Robot 웹 제어기 실행:"
echo "   cd ~/rainbow-robot-connect/RainbowRobot_ConnectTest/src/rb_web"
echo "   python3 rb_web_test.py"
echo ""
