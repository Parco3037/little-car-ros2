"""Start Nav2 and the gated home mission without duplicating robot drivers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    map_file = LaunchConfiguration('map_file').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(map_file):
        raise RuntimeError(
            f'home map is missing: {map_file}; build and save home_mock_v1 before Nav2'
        )
    if not os.path.isfile(params_file):
        raise RuntimeError(f'Nav2 parameter file is missing: {params_file}')

    navigation_share = get_package_share_directory('navigation')
    control_share = get_package_share_directory('raicom_control')
    scan_normalizer = Node(
        package='raicom_control',
        executable='scan_normalizer',
        name='scan_normalizer',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan_stable',
            'samples': 254,
            'angle_min': 0.0,
            'angle_max': 6.283185307179586,
        }],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'bringup.launch.py')
        ),
        launch_arguments={
            'rtabmap': 'false',
            'namespace': '',
            'use_namespace': 'false',
            'map': map_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
            'log_level': 'info',
            'use_teb': 'false',
        }.items(),
    )
    mission = Node(
        package='raicom_control',
        executable='home_mission',
        name='home_mission',
        output='screen',
        parameters=[os.path.join(control_share, 'config', 'home_mock_v1.yaml')],
    )
    nav_cmd_bridge = Node(
        package='raicom_control',
        executable='nav_cmd_bridge',
        name='nav_cmd_bridge',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/raicom/teleop_cmd',
        }],
    )
    # Vendor navigation_base.launch.py indexes this variable directly.
    # Make the safe launcher self-contained instead of relying on an interactive shell.
    compiled_environment = SetEnvironmentVariable('need_compile', 'False')
    return [compiled_environment, scan_normalizer, nav2, nav_cmd_bridge, mission]


def generate_launch_description():
    default_map = '/home/ubuntu/ros2_ws/src/raicom_control/maps/home_mock_v1.yaml'
    default_params = (
        '/home/ubuntu/ros2_ws/src/raicom_control/config/'
        'nav2_params_home_low_load.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        OpaqueFunction(function=launch_setup),
    ])
