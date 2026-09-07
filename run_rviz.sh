#!/bin/bash
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=0
# 시스템 기본 RMW(CycloneDDS)로 뜨면 Nav2(FastRTPS로 강제 설정됨, run_nav2.sh 참고)가 발행하는
# /tf_static(map->odom, latched/TRANSIENT_LOCAL) 같은 latched 토픽을 못 받아 "Frame [map] does not
# exist" 에러가 나는 문제가 있었음 -> Nav2와 동일하게 FastRTPS로 맞춤 (volatile 토픽은 벤더가
# 달라도 문제없지만, TRANSIENT_LOCAL은 크로스벤더 매칭이 불안정한 경우가 있음)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2 -d /home/jeon/t3/isaac/rccar/nav2/limo_view.rviz
