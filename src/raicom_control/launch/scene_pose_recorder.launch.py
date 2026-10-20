"""Start the non-motion fixed-scene pose recorder."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raicom_control',
            executable='scene_pose_recorder',
            name='scene_pose_recorder',
            output='screen',
        ),
    ])
