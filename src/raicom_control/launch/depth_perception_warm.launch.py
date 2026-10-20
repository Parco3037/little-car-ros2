"""Camera/perception/IK only. No servo, hardware driver or grasp action startup."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    def include(package, filename, **kwargs):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', filename)), **kwargs)
    return LaunchDescription([
        SetEnvironmentVariable('need_compile', 'False'),
        SetEnvironmentVariable('MACHINE_TYPE', 'LanderPi_Mecanum'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        include('deptrum-ros-driver-aurora930', 'aurora930_launch.py',
                condition=IfCondition(LaunchConfiguration('start_camera'))),
        include('kinematics', 'kinematics_node.launch.py'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='depth_camera_mount_tf',
             arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam_link', 'depth_camera_link']),
        Node(package='rgbd_grasp', executable='blue_block_3d', name='blue_block_3d_node',
             parameters=[{'target_selection': 'nearest'}], output='screen'),
        Node(package='rgbd_grasp', executable='target_to_ik', name='target_to_ik_node', output='screen'),
    ])
