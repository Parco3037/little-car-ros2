"""Start only the perception-side Aurora observer.

The vendor Aurora driver is intentionally not started here: it is already
owned by the existing MentorPi runtime.  This launch file adds one observer
and therefore cannot duplicate the camera driver or any motion controller.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raicom_control',
            executable='aurora_observer',
            name='aurora_observer',
            output='screen',
        ),
    ])
