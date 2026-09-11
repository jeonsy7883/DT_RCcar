from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[{
                'format': 'UYVY',
                'width': 320,
                'height': 240,
            }],
        ),
        Node(
            package='rccar_driver',
            executable='buzzer_node',
            name='buzzer_node',
        ),
    ])
