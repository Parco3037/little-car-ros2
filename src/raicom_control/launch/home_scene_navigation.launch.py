"""Launch the fixed home-scene navigation and perception chain.

This is intentionally separate from ``home_navigation.launch.py``: the latter
keeps the older navigation-only mission for regression checks, while this
entry point is the one used by keyboard ``M`` and ``scene_mission``.
"""

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
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    map_file = LaunchConfiguration('map_file').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(map_file):
        raise RuntimeError(f'home map is missing: {map_file}')
    if not os.path.isfile(params_file):
        raise RuntimeError(f'Nav2 parameter file is missing: {params_file}')

    navigation_share = get_package_share_directory('navigation')
    control_share = get_package_share_directory('raicom_control')
    mission_config = '/home/ubuntu/ros2_ws/src/raicom_control/config/scene_home_v1.yaml'
    if not os.path.isfile(mission_config):
        mission_config = os.path.join(control_share, 'config', 'scene_home_v1.yaml')
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
    # The vendor bringup currently starts component_container_isolated, which
    # can remain as an empty node on this Pi and never expose composition
    # services. Keep the vendor localization/navigation component descriptions,
    # but own the container here and use the known-working multi-threaded
    # container. This does not modify the vendor workspace.
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_mt',
        parameters=[params_file, {'autostart': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'localization.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'map': map_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
            'container_name': 'nav2_container',
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'navigation_base.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
            'container_name': 'nav2_container',
            # The Pi image has no teb_local_planner plugin. DWB is the
            # installed controller and is the only valid default here.
            'use_teb': 'false',
        }.items(),
    )
    nav_cmd_bridge = Node(
        package='raicom_control',
        executable='nav_cmd_bridge',
        name='nav_cmd_bridge',
        output='screen',
        parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/raicom/teleop_cmd'}],
    )
    scene_mission = Node(
        package='raicom_control',
        executable='scene_mission',
        name='scene_mission',
        output='screen',
        parameters=[mission_config],
    )
    yolo_bridge = Node(
        package='raicom_control',
        executable='yolo_roi_bridge',
        name='yolo_roi_bridge',
        output='screen',
        parameters=[{
            'api_url': ParameterValue(LaunchConfiguration('api_url'), value_type=str),
            'desired_label': ParameterValue(LaunchConfiguration('desired_label'), value_type=str),
            'candidate_roi_json': ParameterValue(
                LaunchConfiguration('candidate_roi_json'), value_type=str,
            ),
            'minimum_score': ParameterValue(
                LaunchConfiguration('minimum_score'), value_type=float,
            ),
        }],
    )
    aurora = Node(
        package='raicom_control',
        executable='aurora_observer',
        name='aurora_observer',
        output='screen',
    )
    # The vendor navigation launcher reads this variable directly.
    compiled_environment = SetEnvironmentVariable('need_compile', 'False')
    return [
        compiled_environment,
        scan_normalizer,
        nav2_container,
        localization,
        navigation,
        nav_cmd_bridge,
        scene_mission,
        yolo_bridge,
        aurora,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/ubuntu/ros2_ws/src/raicom_control/maps/home_mock_v1.yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=(
                '/home/ubuntu/ros2_ws/src/raicom_control/config/'
                'nav2_params_home_low_load.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'api_url', default_value='http://127.0.0.1:8080/api/do_infer',
        ),
        DeclareLaunchArgument('desired_label', default_value=''),
        DeclareLaunchArgument('candidate_roi_json', default_value=''),
        DeclareLaunchArgument('minimum_score', default_value='0.4'),
        OpaqueFunction(function=launch_setup),
    ])
