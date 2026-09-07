#!/bin/bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=0
# CycloneDDS(기본 RMW)에서 controller_server/planner_server가
# "dds_handle_unpin_and_drop_ref" 어설션으로 반복 크래시(exit -6)하는 문제가 있어
# FastDDS로 바꿔서 회피한다. Isaac Sim(rclpy)은 그대로 CycloneDDS를 쓰지만
# RTPS는 표준 프로토콜이라 서로 다른 RMW 구현체끼리도 정상 통신된다.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch /home/jeon/t3/isaac/rccar/nav2/limo_navigation.launch.py
