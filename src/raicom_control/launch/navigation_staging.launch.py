"""Nav2 servers ONLY, with velocity outputs disconnected from the real base.

Requires an existing unique localization/scan/base/guard stack. No mission,
initialpose, actuator node or nav_cmd_bridge is launched here. This is not an
authorized driving entrypoint, even if all lifecycles become active.
"""
from pathlib import Path
import hashlib
import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = Path(get_package_share_directory('raicom_control'))
    params = Path(LaunchConfiguration('params_file').perform(context))
    controller = Path(LaunchConfiguration('controller_file').perform(context))
    for path in (params, controller):
        if not path.is_file():
            raise FileNotFoundError(path)
    # Pi-specific, ABI-preserving bondcpp 3.0.2 backport. Never alter /opt/ros.
    # Fail closed instead of silently returning to the unprotected state reads.
    bond = share/'compat/bondcpp/libbondcpp.so'
    if platform.machine() != 'aarch64' or not bond.is_file():
        raise RuntimeError('Verified aarch64 bondcpp compatibility library required')
    if hashlib.sha256(bond.read_bytes()).hexdigest() != 'd110777939d51480fa4529c68c3b8f220c2b27d0a39cae48a00f052e558e539c':
        raise RuntimeError('bondcpp compatibility library hash mismatch')
    private_env = {
        'LD_LIBRARY_PATH': str(bond.parent)+os.pathsep+os.environ.get('LD_LIBRARY_PATH', ''),
        # Nav2 startup must not starve the production localization gate.  The
        # 2026-09-02 simultaneous start delayed fresh scan/EKF delivery beyond
        # their fail-closed bounds even though the isolated speed path worked.
        'OPENBLAS_NUM_THREADS': '1',
        'OMP_NUM_THREADS': '1',
    }
    names = ['controller_server', 'planner_server', 'smoother_server',
             'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']
    packages = ['nav2_controller', 'nav2_planner', 'nav2_smoother',
                'nav2_behaviors', 'nav2_bt_navigator', 'nav2_waypoint_follower',
                'nav2_velocity_smoother']
    if LaunchConfiguration('fixed_route').perform(context).lower() == 'true':
        # FollowPath alone: no planner, BT, Spin/BackUp, or recovery publisher.
        names = ['controller_server', 'velocity_smoother']
        packages = ['nav2_controller', 'nav2_velocity_smoother']
    # Controller/BT/recovery/smoother acknowledgements must compete normally
    # with the sensor stack.  nice(19) delayed callbacks long enough for BT to
    # misclassify already-accepted goals as timeouts.  Heavy, non-realtime
    # planning/smoothing and waypoint work remains background-priority.
    latency_sensitive = {
        'controller_server', 'behavior_server', 'bt_navigator',
        'velocity_smoother',
    }
    nodes = []
    for name, package in zip(names, packages):
        files = [str(params)]
        if name == 'controller_server':
            files.append(str(controller))
        node_prefix = [] if name in latency_sensitive else ['nice -n 10']
        nodes.append(Node(
            package=package, executable=name, name=name, output='screen',
            additional_env=private_env,
            # Keep every process schedulable on all CPUs.  The 2026-09-02
            # runtime-budget fix limits BT/DWB work in parameters, so the
            # latency-sensitive chain can stay at normal priority without
            # putting planner/smoother work ahead of localization.
            prefix=node_prefix,
            parameters=files+[{'use_sim_time': False}],
            remappings=[('cmd_vel', '/raicom/nav2_staging/raw_cmd_vel'),
                        ('cmd_vel_smoothed', '/raicom/nav2_staging/cmd_vel')],
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        additional_env=private_env,
        prefix=['nice -n 10'],
        parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': names}],
    ))
    # Keep server startup staggered to avoid starving localization. The manager
    # can start immediately: lifecycle service discovery waits for each server,
    # so an additional fixed timer adds latency without proving readiness.
    staggered = [TimerAction(period=float(index), actions=[node])
                 for index, node in enumerate(nodes[:-1])]
    staggered.append(nodes[-1])
    return staggered


def generate_launch_description():
    share = Path(get_package_share_directory('raicom_control'))
    return LaunchDescription([
        DeclareLaunchArgument('fixed_route', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share/'config/nav2_params_home_low_load.yaml'),
        ),
        DeclareLaunchArgument(
            'controller_file',
            default_value=str(share/'config/nav2_controller_mecanum.yaml'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
