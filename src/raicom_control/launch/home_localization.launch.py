"""Fixed-map localization only; no navigation, base bringup or actuator commands."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from raicom_control.map_preflight import validate_saved_map


def _launch_setup(context):
    # Fail closed before constructing any localization node.
    map_file = validate_saved_map(LaunchConfiguration('map_file').perform(context))
    params_file = Path(get_package_share_directory('raicom_control')) / 'config' / 'localization_home.yaml'
    if not params_file.is_file():
        raise FileNotFoundError(params_file)
    return [
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[str(params_file), {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=[str(params_file)],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': False, 'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=(
                '/home/ubuntu/ros2_ws/src/slam/maps/'
                'map_01_2cm_20260901_0240.yaml'
            ),
            description='Verified saved YAML/PGM pair; /scan must have one existing owner.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
