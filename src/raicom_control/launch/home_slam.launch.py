"""Start only SLAM Toolbox on top of the already-running robot base."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    slam_share = get_package_share_directory('slam')
    # Exact 2.6.8 ABI; only this SLAM child sees the repaired common library.
    heading_lib_dir = '/home/ubuntu/raicom_tools/slam_toolbox_2_6_8_heading_v1/lib'
    if not os.path.isfile(os.path.join(heading_lib_dir, 'libtoolbox_common.so')):
        raise RuntimeError('RAICOM SLAM heading library is missing: ' + heading_lib_dir)
    online_exe = '/home/ubuntu/raicom_tools/slam_online_2_6_8_v1/bin/raicom_online_slam_node'
    use_range_candidate = LaunchConfiguration(
        'use_range_candidate').perform(context).lower() == 'true'
    if use_range_candidate:
        online_exe = (
            '/home/ubuntu/raicom_tools/slam_yaw_current/left45_replay/build/'
            'online_range_candidate'
        )
    if not os.path.isfile(online_exe):
        raise RuntimeError('RAICOM online SLAM executable is missing: ' + online_exe)
    # Same-input motion replay traced false rotation locking to finite values
    # outside LaserScan's declared range. The selected adapter repairs only its
    # private input copy. Finer correlation reduces the remaining raster jitter;
    # this profile is not a declaration of localization/navigation acceptance.
    matching_params = {}
    if use_range_candidate:
        matching_params = {
            'minimum_travel_heading': 0.001,
            'correlation_search_space_smear_deviation': 0.03,
            'correlation_search_space_resolution': 0.005,
            'fine_search_angle_offset': 0.0008726646259971648,
        }
    return [
        Node(
            executable=online_exe,
            name='slam_toolbox', output='screen',
            additional_env={
                'LD_LIBRARY_PATH': heading_lib_dir + ':' + os.environ.get('LD_LIBRARY_PATH', ''),
            },
            parameters=[os.path.join(slam_share, 'config', 'slam.yaml'), {
                'use_map_saver': True,
                'use_sim_time': False,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'resolution': 0.02,
                'max_laser_range': 2.0,
                # Rasterization only: incoming scan rate / matching are unchanged.
                'map_update_interval': 0.5,
                # Match fresh scans; KeepLast(1) replaces waiting frames when
                # processing is slower than the sensor. Never build a FIFO.
                'minimum_time_interval': 0.05,
                'online_max_scan_age': 0.5,
                'online_future_tolerance': 0.05,
                # Positive heading gate: at 2 m, .005 rad is about 1 cm.
                # Same-scan stationary comparison reduced stale yaw correction.
                'minimum_travel_heading': 0.005,
                'transform_publish_period': 0.02,
                **matching_params,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_range_candidate',
            default_value='true',
            description=(
                'Correct invalid finite ranges in the private SLAM input. '
                'False selects the legacy profile for explicit rollback only.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
