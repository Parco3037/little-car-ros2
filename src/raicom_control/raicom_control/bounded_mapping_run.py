"""One-shot, fail-closed motion sequence for a small home mapping run."""

from dataclasses import dataclass
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .mapping_logic import distance_2d, directional_clearance, normalize_angle, yaw_from_quaternion


@dataclass(frozen=True)
class Segment:
    name: str
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    target: float = 0.0
    timeout_sec: float = 0.0
    target_kind: str = 'hold'

    @property
    def moving(self) -> bool:
        return any((self.linear_x, self.linear_y, self.angular_z))


def _motion_sequence(
    linear_speed: float, angular_speed: float, distance: float, sequence_mode: str
) -> list[Segment]:
    hold = lambda name: Segment(name=name, target=0.6, timeout_sec=2.0)
    move = lambda name, x, y: Segment(
        name=name,
        linear_x=x,
        linear_y=y,
        target=distance,
        timeout_sec=max(8.0, distance / linear_speed * 2.5),
        target_kind='distance',
    )
    turn = lambda name, z, angle: Segment(
        name=name,
        angular_z=z,
        target=angle,
        timeout_sec=max(12.0, angle / angular_speed * 1.8),
        target_kind='angle',
    )
    if sequence_mode == 'strafe_only':
        return [
            hold('initial_settle'),
            move('strafe_left', 0.0, linear_speed),
            hold('settle_left'),
            move('strafe_right_to_origin', 0.0, -linear_speed),
            hold('final_settle'),
        ]
    if sequence_mode == 'forward_only':
        return [
            hold('initial_settle'),
            move('forward', linear_speed, 0.0),
            hold('final_settle'),
        ]
    if sequence_mode == 'backward_only':
        return [
            hold('initial_settle'),
            move('backward', -linear_speed, 0.0),
            hold('final_settle'),
        ]
    return [
        hold('initial_settle'),
        turn('turn_left_half', angular_speed, math.pi),
        hold('settle_after_left_turn'),
        turn('turn_right_full', -angular_speed, 2.0 * math.pi),
        hold('settle_after_right_turn'),
        turn('turn_left_home', angular_speed, math.pi),
        hold('settle_before_translation'),
        move('forward', linear_speed, 0.0),
        hold('settle_forward'),
        move('backward_to_origin', -linear_speed, 0.0),
        hold('settle_backward'),
        move('strafe_left', 0.0, linear_speed),
        hold('settle_left'),
        move('strafe_right_to_origin', 0.0, -linear_speed),
        hold('settle_center'),
        move('strafe_right', 0.0, -linear_speed),
        hold('settle_right'),
        move('strafe_left_to_origin', 0.0, linear_speed),
        hold('final_settle'),
    ]


class BoundedMappingRun(Node):
    def __init__(self) -> None:
        super().__init__('bounded_mapping_run')
        self.declare_parameter('armed', False)
        self.declare_parameter('command_topic', '/raicom/teleop_cmd')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('sequence_mode', 'full')
        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('angular_speed', 0.20)
        self.declare_parameter('translation_target_m', 0.25)
        self.declare_parameter('max_radius_m', 0.50)
        self.declare_parameter('front_stop_distance_m', 0.35)
        self.declare_parameter('data_timeout_sec', 0.50)

        self.armed = bool(self.get_parameter('armed').value)
        linear_speed = float(self.get_parameter('linear_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)
        distance = float(self.get_parameter('translation_target_m').value)
        self.max_radius = float(self.get_parameter('max_radius_m').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance_m').value)
        self.data_timeout = float(self.get_parameter('data_timeout_sec').value)
        if not 0.0 < linear_speed <= 0.06:
            raise ValueError('linear_speed must be in (0, 0.06] m/s')
        if not 0.0 < angular_speed <= 0.25:
            raise ValueError('angular_speed must be in (0, 0.25] rad/s')
        if not 0.05 <= distance <= 0.30:
            raise ValueError('translation_target_m must be between 0.05 and 0.30 m')
        if not distance < self.max_radius <= 0.60:
            raise ValueError('max_radius_m must exceed target and be no more than 0.60 m')
        if not 0.25 <= self.front_stop_distance <= 0.60:
            raise ValueError('front_stop_distance_m must be between 0.25 and 0.60 m')
        if not 0.20 <= self.data_timeout <= 1.0:
            raise ValueError('data_timeout_sec must be between 0.20 and 1.0 s')

        command_topic = str(self.get_parameter('command_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        sequence_mode = str(self.get_parameter('sequence_mode').value)
        if sequence_mode not in ('full', 'strafe_only', 'forward_only', 'backward_only'):
            raise ValueError(
                'sequence_mode must be full, strafe_only, forward_only or backward_only'
            )
        self.publisher = self.create_publisher(Twist, command_topic, 10)
        self.status_publisher = self.create_publisher(String, '/raicom/mapping/status', 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self._tick)

        self.sequence = _motion_sequence(linear_speed, angular_speed, distance, sequence_mode)
        self.segment_index = 0
        self.segment_started_at: float | None = None
        self.segment_x = 0.0
        self.segment_y = 0.0
        self.segment_yaw = 0.0
        self.origin_x: float | None = None
        self.origin_y: float | None = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.unwrapped_yaw = 0.0
        self.previous_yaw: float | None = None
        self.previous_pose_x: float | None = None
        self.previous_pose_y: float | None = None
        self.last_odom_at: float | None = None
        self.last_scan_at: float | None = None
        self.last_pose_at: float | None = None
        self.scan: LaserScan | None = None
        self.finished = False
        self.succeeded = False
        self.result = 'not started'
        self._waiting_reported = False

        if not self.armed:
            self._finish(False, 'armed=false; no motion permitted')
        else:
            self.get_logger().warn(
                f'ARMED bounded mapping run: target={distance:.2f} m, '
                f'radius_limit={self.max_radius:.2f} m, mode={sequence_mode}'
            )

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        self.last_odom_at = now
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._accept_tracking_pose(
            float(position.x),
            float(position.y),
            yaw_from_quaternion(orientation.z, orientation.w),
            now,
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        self.last_scan_at = time.monotonic()

    def _accept_tracking_pose(self, pose_x: float, pose_y: float, yaw: float, now: float) -> None:
        if self.previous_pose_x is not None and self.previous_pose_y is not None:
            jump = distance_2d(self.previous_pose_x, self.previous_pose_y, pose_x, pose_y)
            yaw_jump = abs(normalize_angle(yaw - (self.previous_yaw or 0.0)))
            if self.segment_started_at is not None and (jump > 0.15 or yaw_jump > 0.35):
                self._finish(
                    False,
                    f'odometry pose jump: translation={jump:.3f} m yaw={yaw_jump:.3f} rad',
                )
                return
        if self.previous_yaw is not None:
            self.unwrapped_yaw += normalize_angle(yaw - self.previous_yaw)
        self.previous_yaw = yaw
        self.previous_pose_x = pose_x
        self.previous_pose_y = pose_y
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.yaw = yaw
        self.last_pose_at = now
        if self.origin_x is None:
            self.origin_x = pose_x
            self.origin_y = pose_y

    def _publish(self, segment: Segment | None = None) -> None:
        msg = Twist()
        if segment is not None:
            msg.linear.x = segment.linear_x
            msg.linear.y = segment.linear_y
            msg.angular.z = segment.angular_z
        self.publisher.publish(msg)

    def publish_zero(self) -> None:
        self._publish(None)

    def _status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)
        self.get_logger().info(text)

    def _finish(self, succeeded: bool, result: str) -> None:
        self.publish_zero()
        self.succeeded = succeeded
        self.result = result
        self.finished = True
        level = self.get_logger().info if succeeded else self.get_logger().error
        level(result)
        self._status(f'{"SUCCEEDED" if succeeded else "ABORTED"}: {result}')

    def _data_fresh(self, now: float) -> bool:
        if self.last_odom_at is None or self.last_scan_at is None or self.last_pose_at is None:
            if not self._waiting_reported:
                self.get_logger().info('waiting for odometry, laser scan and motion tracking pose')
                self._waiting_reported = True
            return False
        return (
            now - self.last_odom_at <= self.data_timeout
            and now - self.last_scan_at <= self.data_timeout
            and now - self.last_pose_at <= self.data_timeout
        )

    def _front_clear(self) -> bool:
        if self.scan is None:
            return False
        clearance, count = directional_clearance(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            0.0,
            math.radians(25.0),
            self.scan.range_min,
            self.scan.range_max,
        )
        if count < 5 or clearance is None:
            self._finish(False, 'front laser sector has insufficient valid samples')
            return False
        if clearance < self.front_stop_distance:
            self._finish(False, f'front obstacle at {clearance:.3f} m')
            return False
        return True

    def _start_segment(self, segment: Segment, now: float) -> None:
        self.segment_started_at = now
        self.segment_x = self.pose_x
        self.segment_y = self.pose_y
        self.segment_yaw = self.unwrapped_yaw
        self._status(f'START {self.segment_index + 1}/{len(self.sequence)} {segment.name}')

    def _segment_complete(self, segment: Segment, now: float) -> bool:
        elapsed = now - (self.segment_started_at or now)
        if elapsed > segment.timeout_sec:
            self._finish(False, f'segment timeout: {segment.name}')
            return False
        if segment.target_kind == 'hold':
            return elapsed >= segment.target
        if segment.target_kind == 'distance':
            return distance_2d(self.segment_x, self.segment_y, self.pose_x, self.pose_y) >= segment.target
        if segment.target_kind == 'angle':
            return abs(self.unwrapped_yaw - self.segment_yaw) >= segment.target
        self._finish(False, f'unknown target kind: {segment.target_kind}')
        return False

    def _tick(self) -> None:
        if self.finished:
            self.publish_zero()
            return
        now = time.monotonic()
        if not self._data_fresh(now):
            self.publish_zero()
            if (self.last_odom_at is not None and now - self.last_odom_at > self.data_timeout) or (
                self.last_scan_at is not None and now - self.last_scan_at > self.data_timeout
            ) or (
                self.last_pose_at is not None and now - self.last_pose_at > self.data_timeout
            ):
                self._finish(False, 'odometry, laser or map tracking data became stale')
            return
        self._waiting_reported = False
        if self.origin_x is None or self.origin_y is None:
            self.publish_zero()
            return
        radius = distance_2d(self.origin_x, self.origin_y, self.pose_x, self.pose_y)
        if radius > self.max_radius:
            self._finish(False, f'radius limit exceeded: {radius:.3f} m')
            return
        if self.segment_index >= len(self.sequence):
            self._finish(True, f'mapping motion complete; final radius={radius:.3f} m')
            return

        segment = self.sequence[self.segment_index]
        if self.segment_started_at is None:
            self._start_segment(segment, now)
        if self._segment_complete(segment, now):
            self.publish_zero()
            self.segment_index += 1
            self.segment_started_at = None
            return
        if self.finished:
            return
        if segment.linear_x > 0.0 and not self._front_clear():
            return
        self._publish(segment)


def main(args=None) -> None:
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BoundedMappingRun()
    try:
        while rclpy.ok() and not stop_requested and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        stop_requested = True
    finally:
        if stop_requested and not node.finished:
            node._finish(False, 'operator or process stop requested')
        if rclpy.ok():
            for _ in range(10):
                node.publish_zero()
                rclpy.spin_once(node, timeout_sec=0.02)
        succeeded = node.succeeded
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if succeeded else 2)


if __name__ == '__main__':
    main()
