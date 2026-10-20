"""Compatibility launch file for the vendor teleop command.

The vendor entry point is kept, but its implementation is routed through the
RAICOM keyboard node so every manual command is checked by cmd_vel_guard.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="raicom_control",
                executable="keyboard_input",
                name="teleop_key_control",
                output="screen",
                parameters=[
                    {"linear_speed": 0.25},
                    {"angular_speed": 0.35},
                    {"command_hold_ms": 200},
                ],
            ),
            Node(
                package="raicom_control",
                executable="manual_control_panel",
                name="manual_control_panel",
                output="screen",
            ),
        ]
    )
