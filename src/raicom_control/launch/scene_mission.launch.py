"""Launch the calibration-gated fixed-scene mission node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raicom_control')
    return LaunchDescription([
        Node(
            package='raicom_control',
            executable='scene_mission',
            name='scene_mission',
            output='screen',
            parameters=[os.path.join(share, 'config', 'scene_home_v1.yaml')],
        ),
    ])
