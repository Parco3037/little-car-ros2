#!/usr/bin/env python3
"""Launch the existing Aurora930 driver and guarded blue-block grasp stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    motion_enabled = LaunchConfiguration('motion_enabled')
    auto_prepare = LaunchConfiguration('auto_prepare')
    auto_grasp = LaunchConfiguration('auto_grasp')
    start_camera = LaunchConfiguration('start_camera')
    start_hardware = LaunchConfiguration('start_hardware')
    start_servo_controller = LaunchConfiguration('start_servo_controller')
    start_description = LaunchConfiguration('start_description')
    target_selection = LaunchConfiguration('target_selection')
    grasp_height_offset_m = LaunchConfiguration('grasp_height_offset_m')
    grasp_id4_offset_pulse = LaunchConfiguration('grasp_id4_offset_pulse')
    sag_compensation_m = LaunchConfiguration('sag_compensation_m')
    pregrasp_clearance_m = LaunchConfiguration('pregrasp_clearance_m')

    camera_launch = os.path.join(
        get_package_share_directory('deptrum-ros-driver-aurora930'),
        'launch',
        'aurora930_launch.py',
    )
    kinematics_launch = os.path.join(
        get_package_share_directory('kinematics'),
        'launch',
        'kinematics_node.launch.py',
    )
    hardware_launch = os.path.join(
        get_package_share_directory('ros_robot_controller'),
        'launch',
        'ros_robot_controller.launch.py',
    )
    servo_launch = os.path.join(
        get_package_share_directory('servo_controller'),
        'launch',
        'servo_controller.launch.py',
    )
    description_launch = os.path.join(
        get_package_share_directory('raicom_control'),
        'launch',
        'lightweight_robot_description.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_selection', default_value='nearest',
                              description='nearest selects the closest depth group, then its upper cube; min_v/max_v use image height'),
        DeclareLaunchArgument('grasp_height_offset_m', default_value='0.015',
                              description='Raise the grasp path in arm-world Z, in metres (0 to 0.020)'),
        DeclareLaunchArgument('sag_compensation_m', default_value='0.020',
                              description='Extra world-Z allowance for arm sag, metres (0 to 0.040)'),
        DeclareLaunchArgument('pregrasp_clearance_m', default_value='0.015',
                              description='Additional height during pregrasp and first approach half (0 to 0.030)'),
        DeclareLaunchArgument('grasp_id4_offset_pulse', default_value='60',
                              description='Add ID4 pulses only to grasp approach commands (0 to 60)'),
        DeclareLaunchArgument(
            'motion_enabled',
            default_value='true',
            description='Allow the grasp stack to move servos',
        ),
        DeclareLaunchArgument(
            'auto_prepare',
            default_value='true',
            description='Move once to the configured observation pose at launch',
        ),
        DeclareLaunchArgument(
            'auto_grasp',
            default_value='true',
            description='Run exactly one grasp after automatic prepare',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start Aurora930 only when no driver is already running',
        ),
        DeclareLaunchArgument(
            'start_hardware',
            default_value='false',
            description='Start hardware only when the vendor base stack is absent',
        ),
        DeclareLaunchArgument(
            'start_servo_controller',
            default_value='false',
            description='Start the servo command forwarding node',
        ),
        DeclareLaunchArgument(
            'start_description',
            default_value='false',
            description='Publish joint states and the base-to-camera TF chain',
        ),
        SetEnvironmentVariable('need_compile', 'False'),
        SetEnvironmentVariable('MACHINE_TYPE', 'LanderPi_Mecanum'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            condition=IfCondition(start_hardware),
        ),
        TimerAction(
            period=1.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(servo_launch),
                condition=IfCondition(start_servo_controller),
                launch_arguments={'base_frame': 'base_link'}.items(),
            )],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=IfCondition(start_description),
            launch_arguments={
                'use_sim_time': 'false',
                'frame_prefix': '',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(start_camera),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinematics_launch),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_camera_mount_tf',
            arguments=[
                '0', '0', '0',
                '-1.57', '0', '-1.57',
                'depth_cam_link',
                'depth_camera_link',
            ],
        ),
        Node(
            package='rgbd_grasp',
            executable='blue_block_3d',
            name='blue_block_3d_node',
            output='screen',
            parameters=[{'target_selection': target_selection}],
        ),
        Node(
            package='rgbd_grasp',
            executable='target_to_ik',
            name='target_to_ik_node',
            output='screen',
        ),
        Node(
            package='rgbd_grasp',
            executable='grasp_once',
            name='grasp_once_node',
            output='screen',
            parameters=[{
                'motion_enabled': motion_enabled,
                'auto_prepare': auto_prepare,
                'auto_grasp': auto_grasp,
                'target_selection': target_selection,
                'grasp_height_offset_m': ParameterValue(grasp_height_offset_m, value_type=float),
                'grasp_id4_offset_pulse': ParameterValue(grasp_id4_offset_pulse, value_type=int),
                'sag_compensation_m': ParameterValue(sag_compensation_m, value_type=float),
                'pregrasp_clearance_m': ParameterValue(pregrasp_clearance_m, value_type=float),
            }],
        ),
    ])
