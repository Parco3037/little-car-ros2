"""Single-owner LanderPi base chain.

This entry starts the existing vendor controller and MS200 launch fragments,
but disables the vendor EKF instance and owns one clean EKF configuration in
this package.  The independent systemd guard remains the sole guard owner;
this launch deliberately does not include a second guard.  It does not
start map localization, Nav2, mission, vision, arm, gripper, or any
experimental fusion process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('raicom_control')
    controller_share = get_package_share_directory('controller')
    peripherals_share = get_package_share_directory('peripherals')
    ekf_file = os.path.join(control_share, 'config', 'ekf_production.yaml')
    controller_launch = os.path.join(
        controller_share, 'launch', 'controller.launch.py')
    lidar_launch = os.path.join(
        peripherals_share, 'launch', 'lidar.launch.py')
    for path in (ekf_file, controller_launch, lidar_launch):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

    return LaunchDescription([
        # The vendor launch fragments read these environment variables directly.
        SetEnvironmentVariable('need_compile', 'False'),
        SetEnvironmentVariable('LIDAR_TYPE', 'MS200'),
        SetEnvironmentVariable('MACHINE_TYPE', 'LanderPi_Mecanum'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={
                'enable_odom': 'false',
                'use_namespace': 'false',
                'namespace': '',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'imu_frame': 'imu_link',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={
                'lidar_frame': 'lidar_frame',
                'scan_raw': 'scan_raw',
                'scan_topic': 'scan',
            }.items(),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('odometry/filtered', 'odom'),
                ('cmd_vel', 'controller/cmd_vel'),
            ],
        ),
    ])
