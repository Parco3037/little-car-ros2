"""Execute one guarded Nav2 point on an existing verified map.

This operational helper starts localization and RViz, performs a read-only
scan/map overlap check, and only then sends one NavigateToPose goal.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


MAP_DEFAULT = (
    '/home/ubuntu/ros2_ws/src/slam/maps/'
    'map_01_2cm_20260901_0240.yaml'
)
CONFIG_DIRS = [
    '/home/ubuntu/ros2_ws/src/raicom_control/config',
    '/home/ubuntu/ros2_ws/install/raicom_control/share/raicom_control/config',
]
NAV_RVIZ = (
    '/home/ubuntu/ros2_ws/src/raicom_control/config/'
    'localization_watch_1fps.rviz'
)


def stop_process(process, name):
    if process is None or process.poll() is not None:
        return
    try:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4.0)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=2.0)
    except (OSError, subprocess.TimeoutExpired):
        pass


def launch(argv, log):
    return subprocess.Popen(
        argv,
        stdout=log,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )


def wait_for(check, timeout_s, interval_s=0.1, message='condition'):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if check():
            return True
        time.sleep(interval_s)
    raise TimeoutError(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--initial-x', type=float, required=True)
    parser.add_argument('--initial-y', type=float, required=True)
    parser.add_argument('--initial-yaw-deg', type=float, required=True)
    parser.add_argument('--target', default='recognition')
    parser.add_argument('--align-seconds', type=float, default=10.0)
    parser.add_argument('--align-only', action='store_true')
    parser.add_argument('--expected-boot', required=True)
    args = parser.parse_args()
    import fcntl
    operator_lock = open('/tmp/raicom_operator_session.lock', 'a')
    fcntl.flock(operator_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    from raicom_control.field_tool_model import (
        migrate_legacy_route,
        navigation_pose,
        read_route_file,
        route_file_for_map,
    )
    from raicom_control.field_tool_ros import (
        FieldToolNode,
        NavigationController,
        NavigationTarget,
    )

    rclpy.init()
    node = FieldToolNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    route_file = route_file_for_map(args.map, node.route_dir)
    route = (read_route_file(route_file, args.map) if route_file.exists()
             else migrate_legacy_route(args.map, CONFIG_DIRS))
    if route is None:
        raise RuntimeError('existing route is unavailable')
    if args.target not in route['points']:
        raise RuntimeError('target point is unavailable')
    initial = dict(x=args.initial_x, y=args.initial_y,
                   yaw_rad=math.radians(args.initial_yaw_deg))
    target = navigation_pose(route['points'][args.target])
    if node.boot_id != args.expected_boot:
        raise RuntimeError('Pi boot changed before P0 execution')

    session = Path(
        '/home/ubuntu/raicom_runtime/p0_single_nav_'
        + time.strftime('%Y%m%dT%H%M%S')
    )
    session.mkdir(parents=True, exist_ok=True)
    localization_log = (session / 'localization.log').open('wb')
    rviz_log = (session / 'rviz.log').open('wb')
    nav_log = (session / 'nav2.log').open('wb')
    alignment_log = (session / 'alignment.log').open('wb')
    localization = None
    rviz = None
    nav2 = None
    try:
        localization = launch(
            [
                'ros2', 'launch', 'raicom_control',
                'home_localization.launch.py',
                'map_file:=' + str(Path(args.map).resolve()),
            ],
            localization_log,
        )
        rviz = launch(
            [
                'dbus-run-session', '--', 'nice', '-n', '10',
                'rviz2', '-d', NAV_RVIZ,
            ],
            rviz_log,
        )

        def spin(seconds=0.02):
            time.sleep(seconds)

        def ready_map():
            spin()
            return node.map_message is not None

        def ready_initial_pose():
            spin()
            return (
                node.map_message is not None
                and node.initial_pose_subscriber_ready()
            )

        wait_for(ready_map, 20.0, message='map_server did not publish /map')
        wait_for(
            ready_initial_pose, 15.0,
            message='AMCL did not subscribe to /initialpose',
        )
        for _ in range(3):
            node.publish_initial_pose(
                initial['x'], initial['y'], initial['yaw_rad'])
            time.sleep(0.5)
            spin(0.02)

        def fixed_tf():
            spin()
            return node.fixed_tf_ok(0.5) and node.amcl_pose is not None

        wait_for(
            fixed_tf, 20.0,
            message='fixed map TF or /amcl_pose did not become valid',
        )

        verifier = launch(
            [
                'python3', '-B',
                '/home/ubuntu/ros2_ws/src/raicom_control/deploy/'
                'verify_scan_map_alignment.py',
                '--seconds', str(args.align_seconds),
            ],
            alignment_log,
        )
        try:
            verifier.wait(timeout=args.align_seconds + 15.0)
        except subprocess.TimeoutExpired:
            stop_process(verifier, 'alignment-verifier')
            raise RuntimeError('scan/map alignment verifier timed out')
        alignment_output = alignment_log.name
        alignment_data = json.loads(
            Path(alignment_output).read_text(encoding='utf-8') or '{}')
        frames = alignment_data.get('frames', [])
        if len(frames) < 3:
            raise RuntimeError(
                'scan/map alignment returned too few frames: '
                + str(len(frames)))
        failed_frames = [
            frame for frame in frames
            if not frame.get('all_visible', {}).get('trial_criteria_met')
        ]
        if failed_frames:
            raise RuntimeError(
                'scan/map alignment gate failed on '
                f'{len(failed_frames)}/{len(frames)} frames')

        if args.align_only:
            return 0

        nav2 = launch(
            [
                'ros2', 'launch', 'raicom_control',
                'field_tool_navigation.launch.py',
            ],
            nav_log,
        )
        wait_for(
            lambda: (spin(), node.navigate_client.server_is_ready())[1],
            90.0,
            interval_s=0.2,
            message='NavigateToPose action server did not become ready',
        )
        target_spec = NavigationTarget(
            name=args.target,
            label=str(route['points'][args.target]['label']),
            x=target['x'],
            y=target['y'],
            yaw=target['yaw_rad'],
            position_tolerance_m=route['points'][args.target][
                'position_tolerance_m'],
            yaw_tolerance_deg=route['points'][args.target][
                'yaw_tolerance_deg'],
            timeout_s=120.0,
        )
        mission_id = 'p0-single-' + time.strftime('%Y%m%dT%H%M%S')
        controller = node.navigation
        controller.start([target_spec], mission_id)
        global_deadline = time.monotonic() + 135.0
        start_pose = node.lookup_pose('map', 'base_footprint')
        max_displacement = 0.0
        status = None
        while controller.state not in {
                controller.SUCCEEDED, controller.FAILED, controller.STOPPED}:
            spin(0.02)
            controller.poll()
            try:
                pose = node.lookup_pose('map', 'base_footprint', timeout_s=0.2)
            except Exception:
                pose = None
            if pose is not None:
                max_displacement = max(
                    max_displacement,
                    math.hypot(
                        pose[0] - start_pose[0],
                        pose[1] - start_pose[1],
                    ),
                )
            if time.monotonic() > global_deadline:
                controller.stop()
                raise RuntimeError('P0 navigation controller global timeout')
            status = controller.state
        if status != controller.SUCCEEDED:
            raise RuntimeError(
                'P0 navigation did not succeed: '
                + str(controller.last_error))
        report = {
            'schema': 'raicom_p0_single_nav/v1',
            'boot_id': node.boot_id,
            'map': str(Path(args.map).resolve()),
            'initial_pose_operator_supplied': initial,
            'target': args.target,
            'target_pose': {
                'x': target['x'],
                'y': target['y'],
                'yaw_deg': target['yaw_deg'],
            },
            'alignment_frames': len(frames),
            'max_displacement_m': max_displacement,
            'action_state': status,
        }
        (session / 'result.json').write_text(
            json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(report, indent=2))
        return 0
    finally:
        node.emergency_stop()
        stop_process(nav2, 'nav2')
        stop_process(rviz, 'rviz')
        stop_process(localization, 'localization')
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for stream in (localization_log, rviz_log, nav_log, alignment_log):
            stream.close()
        operator_lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
