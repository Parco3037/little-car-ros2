"""Start only the safety-gated bridge for an already-audited Nav2 staging stack.

This launch does not start Nav2, localization, a goal client or any actuator.
It exists so operators do not need to hand-build a sourced ``ros2 run``
command or redirect logs into an accidentally unwritable directory.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raicom_control',
            executable='nav_cmd_bridge',
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
    ])
