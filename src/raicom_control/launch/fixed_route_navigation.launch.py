"""Opt-in fixed FollowPath servers and the existing guarded bridge; no goal."""
import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(_context):
    import rclpy
    rclpy.init()
    probe=rclpy.create_node('fixed_route_launch_preflight')
    try:
        until=time.monotonic()+1.
        while time.monotonic()<until:rclpy.spin_once(probe,timeout_sec=.05)
        conflicts=set(probe.get_node_names()) & {'controller_server','velocity_smoother',
            'nav_cmd_bridge','planner_server','behavior_server','bt_navigator','raicom_field_tool'}
        if conflicts:raise RuntimeError('Stop existing navigation/operator session before switching: '+str(sorted(conflicts)))
        owners=probe.get_publishers_info_by_topic('/controller/cmd_vel')
        if len(owners)!=1 or owners[0].node_name!='cmd_vel_guard':raise RuntimeError('Unique existing guard required')
    finally:probe.destroy_node();rclpy.shutdown()
    share = get_package_share_directory('raicom_control')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'navigation_staging.launch.py')),
            launch_arguments={
                'fixed_route': 'true',
                'params_file': os.path.join(share, 'config', 'nav2_fixed_route.yaml'),
                'controller_file': os.path.join(share, 'config', 'nav2_controller_fixed.yaml'),
            }.items()),
        Node(executable='/usr/bin/python3',
             arguments=['-m', 'raicom_control.field_tool_nav_bridge'],
             name='nav_cmd_bridge', output='screen', parameters=[{
                 'input_topic': '/raicom/nav2_staging/cmd_vel',
                 'output_topic': '/raicom/teleop_cmd',
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
