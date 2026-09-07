"""
Limo Nav2 브링업 (ground truth localization, AMCL 없음).

- map_server (limo_map.yaml) + 전용 lifecycle_manager
- static_transform_publisher로 map -> odom identity 고정
  (Isaac Sim이 odom -> base_link를 ground truth pose로 직접 publish하므로 별도 localization 불필요)
- nav2_bringup의 navigation_launch.py (controller/smoother/planner/behavior/bt_navigator/
  waypoint_follower/velocity_smoother + 이들의 lifecycle_manager, AMCL/SLAM 제외)

실행:
    ros2 launch /home/jeon/t3/isaac/rccar/nav2/limo_navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_YAML = os.path.join(THIS_DIR, "limo_map.yaml")
PARAMS_FILE = os.path.join(THIS_DIR, "nav2_params_limo.yaml")


def generate_launch_description():
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": MAP_YAML, "use_sim_time": True}],
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"], "use_sim_time": True}],
    )

    # ground truth localization: map == odom (identity), Isaac Sim이 odom->base_link를 직접 publish
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    # limo_nav2_behavior.py가 odom->base_link는 발행하지만 라이다 장착 위치(base_link->lidar_frame)는
    # 발행하지 않아서 /scan(frame_id=lidar_frame)이 RViz/costmap에서 계속 버려지던 문제 보완.
    # 값은 prepare_nav2_assets_limo.py에서 RTX Lidar를 /limo/lidar에 생성할 때 쓴
    # translation=(0,0,0.3), orientation identity 와 동일 (limo가 base_link와 원점 일치한다고 가정).
    base_to_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar",
        output="screen",
        arguments=["0", "0", "0.3", "0", "0", "0", "base_link", "lidar_frame"],
        parameters=[{"use_sim_time": True}],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": PARAMS_FILE,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            map_server_node,
            lifecycle_manager_map,
            map_to_odom_tf,
            base_to_lidar_tf,
            navigation_launch,
        ]
    )
