#!/bin/bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=0
# 시스템 ROS2(Python 3.10용 rclpy)가 Isaac Sim의 Python 3.11과 충돌해서
# "No module named 'rclpy._rclpy_pybind11'" 에러가 나므로, Isaac Sim 자체 내장
# ROS2 브릿지가 자기 버전을 쓰도록 PYTHONPATH를 비운다.
unset PYTHONPATH
cd /home/jeon/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
./isaac-sim.sh "/home/jeon/t3/isaac/rccar/0907.usd" --/app/scripting/ignoreWarningDialog=true
