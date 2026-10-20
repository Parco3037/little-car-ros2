from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raicom_control',
            executable='cmd_vel_guard',
            name='cmd_vel_guard',
            output='screen',
            parameters=[{
                'input_topic': '/raicom/teleop_cmd',
                'output_topic': '/controller/cmd_vel',
                'timeout_ms': 250,
                'publish_rate_hz': 20.0,
                'max_linear': 0.25,
                'max_angular': 0.35,
                'max_linear_accel': 0.5,
                'max_angular_accel': 0.7,
                'motion_enable_topic': '/raicom/motion_enable',
                'stop_topic': '/raicom/nav_stop',
            }],
        ),
    ])
