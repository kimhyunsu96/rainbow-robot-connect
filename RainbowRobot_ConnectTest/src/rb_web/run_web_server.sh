#!/bin/bash
# Rainbow Robot Web Controller 실행 스크립트

cd "$(dirname "$0")"

# Python 패키지 설치 (선택사항)
# pip install -r requirements.txt

# ROS2 설정
source /opt/ros/humble/setup.bash

# 웹 서버 실행
python3 rb_web_test.py
