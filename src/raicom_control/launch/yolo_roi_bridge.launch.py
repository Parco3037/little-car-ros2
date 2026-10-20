"""Start the on-demand semantic ROI bridge only.

The fixed home scene normally contains both class 1 (gear) and class 2
(screw). The launch arguments make the selected target explicit and keep
ambiguous frames fail-closed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'api_url', default_value='http://127.0.0.1:8080/api/do_infer',
        ),
        DeclareLaunchArgument('desired_label', default_value=''),
        DeclareLaunchArgument('candidate_roi_json', default_value=''),
        DeclareLaunchArgument('minimum_score', default_value='0.4'),
        Node(
            package='raicom_control',
            executable='yolo_roi_bridge',
            name='yolo_roi_bridge',
            output='screen',
            parameters=[{
                'api_url': ParameterValue(LaunchConfiguration('api_url'), value_type=str),
                'desired_label': ParameterValue(LaunchConfiguration('desired_label'), value_type=str),
                'candidate_roi_json': ParameterValue(
                    LaunchConfiguration('candidate_roi_json'), value_type=str,
                ),
                'minimum_score': ParameterValue(
                    LaunchConfiguration('minimum_score'), value_type=float,
                ),
            }],
        ),
    ])
