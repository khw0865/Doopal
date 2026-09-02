from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('assistive_detection'))
    hand_params = str(package_share / 'config' / 'hand_nodes.yaml')

    return LaunchDescription([
        Node(
            package='assistive_detection',
            executable='hand_detection',
            name='mediapipe_palm_3d_action_server',
            output='screen',
            parameters=[hand_params],
        ),
        Node(
            package='assistive_detection',
            executable='hand_tracking',
            name='realsense_hand_image_servo_follow_service',
            output='screen',
            parameters=[hand_params],
        ),
        Node(
            package='assistive_detection',
            executable='object_detection',
            name='object_detection_node',
            output='screen',
        ),
    ])
