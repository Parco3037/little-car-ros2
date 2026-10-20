"""Expose existing HTTP containers as ROS2 nodes; no camera driver or Docker mutation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters={name:ParameterValue(LaunchConfiguration(name),value_type=str) for name in ('camera_url','infer_url')}
    return LaunchDescription([
        DeclareLaunchArgument('camera_url',default_value='http://127.0.0.1:8000'),
        DeclareLaunchArgument('infer_url',default_value='http://127.0.0.1:8080'),
        Node(package='raicom_control',executable='yolo_camera',name='yolo_camera',parameters=[parameters],output='screen'),
        Node(package='raicom_control',executable='yolo_infer',name='yolo_infer',parameters=[parameters],output='screen'),
    ])
