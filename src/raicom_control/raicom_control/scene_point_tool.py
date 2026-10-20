"""Operator-facing, non-motion tool for inspecting and recording scene poses.

The tool deliberately has no velocity, servo, or action publishers.  ``status``
only reads the timestamped ``map -> base_footprint`` transform and the latest
guarded velocity.  ``record_pick``/``record_place`` call the already gated
pose-recorder services; the recorder performs the final zero-speed and TF
freshness checks before editing the scene YAML.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_ros

from .scene_pose_recorder_logic import quaternion_to_yaw


class ScenePointTool(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('scene_point_tool')
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.tf_timeout_sec = args.tf_timeout_sec
        self.pose_max_age_sec = args.pose_max_age_sec
        self.velocity_max_linear_mps = args.velocity_max_linear_mps
        self.velocity_max_angular_rps = args.velocity_max_angular_rps
        self.velocity_topic = args.velocity_topic
        self._velocity: Twist | None = None
        self._velocity_at: float | None = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(Twist, self.velocity_topic, self._on_velocity, 10)

    def _on_velocity(self, message: Twist) -> None:
        self._velocity = message
        self._velocity_at = time.monotonic()

    def _lookup_pose(self) -> tuple[float, float, float, float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            raise RuntimeError(
                f'缺少 {self.map_frame} -> {self.base_frame} 定位 TF；'
                '请先启动固定地图定位并完成初始位姿设置'
            ) from exc
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1_000_000_000.0
        now_ros = self.get_clock().now().nanoseconds / 1_000_000_000.0
        age = now_ros - stamp
        if stamp <= 0.0 or age < 0.0 or age > self.pose_max_age_sec:
            raise RuntimeError(
                f'定位 TF 已过期（age={age:.3f}s，限制={self.pose_max_age_sec:.3f}s）'
            )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return float(translation.x), float(translation.y), yaw, max(0.0, age)

    def _velocity_state(self) -> tuple[str, float, float]:
        if self._velocity is None or self._velocity_at is None:
            return 'missing', float('inf'), float('inf')
        age = time.monotonic() - self._velocity_at
        if age > self.pose_max_age_sec:
            return 'stale', age, float('inf')
        message = self._velocity
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            return 'nonfinite', age, float('inf')
        linear = max(abs(value) for value in values[:3])
        angular = max(abs(value) for value in values[3:])
        stopped = linear <= self.velocity_max_linear_mps and angular <= self.velocity_max_angular_rps
        return ('zero' if stopped else 'moving'), linear, angular

    def print_status(self) -> bool:
        try:
            x, y, yaw, age = self._lookup_pose()
            pose_text = f'x={x:.3f} m, y={y:.3f} m, yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg), tf_age={age:.3f} s'
            pose_ok = True
        except RuntimeError as exc:
            pose_text = str(exc)
            pose_ok = False
        state, first, second = self._velocity_state()
        if state in {'zero', 'moving'}:
            velocity_text = f'{state}: linear={first:.4f} m/s, angular={second:.4f} rad/s'
        elif state == 'stale':
            velocity_text = f'stale: age={first:.3f} s'
        else:
            velocity_text = state
        print(f'pose: {pose_text}')
        print(f'velocity({self.velocity_topic}): {velocity_text}')
        if pose_ok and state == 'zero':
            print('input check: pose and zero velocity available; saved map and fixed localization still required before recording')
        else:
            print('record gate: NOT READY; do not record yet')
        return pose_ok and state == 'zero'

    def inputs_ready(self) -> bool:
        """Wait for discovery/data, not for a moving vehicle to become stopped."""
        state, _first, _second = self._velocity_state()
        if state not in {'zero', 'moving'}:
            return False
        # A zero-time readiness check lets the single-threaded executor keep
        # processing TF and velocity while DDS endpoints are being discovered.
        if not self._tf_buffer.can_transform(self.map_frame, self.base_frame, Time()):
            return False
        try:
            self._lookup_pose()
        except RuntimeError:
            return False
        return True

    def record(self, station: str, wait_sec: float) -> bool:
        if station in {'start', 'recognition', 'pickup', 'park_1', 'park_2'}:
            service_name = f'/raicom/route/record_{station}'
        elif station == 'list_route':
            service_name = '/raicom/route/list'
        else:
            service_name = f'/raicom/scene/record_{station}'
        client = self.create_client(Trigger, service_name)
        deadline = time.monotonic() + wait_sec
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                print(f'未找到服务 {service_name}；请先启动 scene_pose_recorder')
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        request = Trigger.Request()
        future = client.call_async(request)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            print(f'调用 {service_name} 超时；写入状态未知，请检查场景 YAML，勿盲目重复调用')
            return False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS transport failure
            print(f'调用 {service_name} 失败：{exc}')
            return False
        prefix = '点位清单：' if station == 'list_route' else '记录成功：'
        print((prefix if response.success else '请求拒绝：') + response.message)
        return bool(response.success)


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='只读查看或记录 home 场景点位；本工具不发布运动/舵机指令'
    )
    parser.add_argument('action', choices=(
        'status', 'record_pick', 'record_place', 'list_route', 'record_start',
        'record_recognition', 'record_pickup', 'record_park_1', 'record_park_2'))
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--velocity-topic', default='/controller/cmd_vel')
    parser.add_argument('--tf-timeout-sec', type=float, default=0.2)
    parser.add_argument('--pose-max-age-sec', type=float, default=1.0)
    parser.add_argument('--velocity-max-linear-mps', type=float, default=0.01)
    parser.add_argument('--velocity-max-angular-rps', type=float, default=0.02)
    parser.add_argument('--wait-sec', type=float, default=8.0,
                        help='每阶段等待 ROS 数据或服务的最大秒数，必须为有限正数')
    parsed = parser.parse_args(argv)
    if not math.isfinite(parsed.wait_sec) or parsed.wait_sec <= 0:
        parser.error('--wait-sec must be a finite positive number')
    return parsed


def main(args: list[str] | None = None) -> None:
    parsed = _parse_args(args)
    rclpy.init(args=args)
    node = ScenePointTool(parsed)
    try:
        # DDS discovery can take more than one second after a fresh CLI starts.
        # Honor the requested budget, but return early once both inputs arrive.
        until = time.monotonic() + parsed.wait_sec
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, until - time.monotonic())))
            if node.inputs_ready():
                break
        if parsed.action == 'list_route':
            success = node.record('list_route', parsed.wait_sec)
        elif parsed.action == 'status':
            success = node.print_status()
        else:
            # This status is informative only; the recorder service is the final gate.
            node.print_status()
            success = node.record(parsed.action.removeprefix('record_'), parsed.wait_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 2)


if __name__ == '__main__':
    main()
