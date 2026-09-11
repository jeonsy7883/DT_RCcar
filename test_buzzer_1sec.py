#!/usr/bin/env python3
"""실물 RC카 부저를 1초간 울리는 테스트 스크립트.

사전 조건:
- 라즈베리파이에 buzzer_node가 떠 있어야 함
  (ssh -i ~/.ssh/rccar_ed25519 ubuntu@172.40.10.134 후
   ros2 run rccar_driver buzzer_node 실행, 또는 아래 안내 참고)
- 데스크탑에서: source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0
  && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp 실행 후 이 스크립트 실행

사용법:
    python3 test_buzzer_1sec.py [duty]
    duty: 0~100 (기본 25, 작을수록 조용함)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

DUTY = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0


def main():
    rclpy.init()
    node = rclpy.create_node("buzzer_test_once")
    pub = node.create_publisher(Float32, "/buzzer_volume", 10)

    print("buzzer_node와 매칭 대기 중...")
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    if pub.get_subscription_count() == 0:
        print("경고: buzzer_node를 못 찾았습니다. 라즈베리파이에서 buzzer_node가 실행 중인지 확인하세요.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"켜기 (duty={DUTY})")
    pub.publish(Float32(data=DUTY))
    time.sleep(1.0)
    print("끄기")
    pub.publish(Float32(data=0.0))
    time.sleep(0.3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
