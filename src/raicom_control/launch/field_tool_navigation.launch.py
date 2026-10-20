"""Start the existing isolated Nav2 stack and bridge for the field tool.

Localization must already be running. No map, AMCL, mission, arm, or
un-gated velocity publisher is started here.
"""

import os
import json

from raicom_control.nav_scan_geometry import navigation_scan_profile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(_context):
    share = get_package_share_directory('raicom_control')
    phased = LaunchConfiguration('workspace_profile').perform(_context).lower() == 'true'
    # Both controller modes use the exact body-only costmap exclusion.
    get_package_share_directory('raicom_nav2_workspace')
    params, footprint = navigation_scan_profile(
        os.path.join(share, 'config',
                     'nav2_params_workspace.yaml' if phased else 'nav2_params_home_straight_low_load.yaml'),
        '/home/ubuntu/raicom_runtime/nav_scan_self_filter')
    return [
        Node(
            executable='/usr/bin/python3',
            arguments=['-m', 'raicom_control.nav_scan_self_filter'],
            name='nav_scan_self_filter', output='screen',
            parameters=[{'footprint': json.dumps(footprint)}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'navigation_staging.launch.py')
            ),
            launch_arguments={
                'params_file': str(params),
                'controller_file': os.path.join(
                    share, 'config', 'nav2_controller_workspace.yaml' if phased else 'nav2_controller_straight.yaml'),
            }.items(),
        ),
        Node(
            executable='/usr/bin/python3',
            arguments=['-m', 'raicom_control.field_tool_nav_bridge'],
            name='nav_cmd_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/raicom/nav2_staging/cmd_vel',
                'output_topic': '/raicom/teleop_cmd',
                'stop_topic': '/raicom/nav_stop',
                'active_topic': '/raicom/navigation_active',
                'manual_suppressed_topic': '/raicom/manual_suppressed',
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('workspace_profile', default_value='false'),
                              OpaqueFunction(function=launch_setup)])
