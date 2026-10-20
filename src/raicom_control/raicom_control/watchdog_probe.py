"""One-shot live watchdog timing probe with a fail-closed arming gate."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class WatchdogProbe(Node):
    def __init__(self) -> None:
        super().__init__('watchdog_probe')
        self.declare_parameter('armed', False)
        self.declare_parameter('command_topic', '/raicom/teleop_cmd')
        self.declare_parameter('output_topic', '/controller/cmd_vel')
        self.declare_parameter('speed_mps', 0.03)
        self.declare_parameter('active_sec', 0.50)
        self.declare_parameter('max_stop_sec', 0.30)

        self.armed = bool(self.get_parameter('armed').value)
        self.speed = float(self.get_parameter('speed_mps').value)
        self.active_sec = float(self.get_parameter('active_sec').value)
        self.max_stop_sec = float(self.get_parameter('max_stop_sec').value)
        if not math.isfinite(self.speed) or not 0.0 < self.speed <= 0.04:
            raise ValueError('speed_mps must be finite and in (0, 0.04]')
        if not 0.20 <= self.active_sec <= 1.0:
            raise ValueError('active_sec must be between 0.20 and 1.0')
        if not 0.25 <= self.max_stop_sec <= 0.30:
            raise ValueError('max_stop_sec must be between 0.25 and 0.30')

        command_topic = str(self.get_parameter('command_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(Twist, command_topic, 1)
        self.subscription = self.create_subscription(Twist, output_topic, self._on_output, 1)
        self.timer = self.create_timer(0.05, self._tick)
        self.started_at = time.monotonic()
        self.active_started_at: float | None = None
        self.released_at: float | None = None
        self.finished = False
        self.succeeded = False
        self.result = 'not started'

        if not self.armed:
            self._finish(False, 'armed=false; no motion permitted')
        else:
            self.get_logger().warn(
                f'ARMED watchdog probe: speed={self.speed:.3f} m/s, '
                f'active={self.active_sec:.2f} s'
            )

    def _publish(self, speed: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = speed
        self.publisher.publish(msg)

    def _finish(self, succeeded: bool, result: str) -> None:
        if self.finished:
            return
        self._publish()
        self.finished = True
        self.succeeded = succeeded
        self.result = result
        log = self.get_logger().info if succeeded else self.get_logger().error
        log(result)

    def _on_output(self, msg: Twist) -> None:
        if self.finished or self.released_at is None:
            return
        is_zero = all(
            abs(value) <= 1.0e-4
            for value in (
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            )
        )
        if not is_zero:
            return
        latency = time.monotonic() - self.released_at
        self._finish(
            latency <= self.max_stop_sec,
            f'watchdog zero-output latency={latency * 1000.0:.1f} ms '
            f'(limit={self.max_stop_sec * 1000.0:.0f} ms)',
        )

    def _tick(self) -> None:
        if self.finished:
            self._publish()
            return
        now = time.monotonic()
        if self.active_started_at is None:
            if now - self.started_at < 0.50:
                self._publish()
                return
            self.active_started_at = now
            self._publish(self.speed)
            return
        if self.released_at is None:
            if now - self.active_started_at < self.active_sec:
                self._publish(self.speed)
                return
            self.released_at = now
            self.get_logger().info('command publication released; waiting for watchdog zero')
            return
        if now - self.released_at > self.max_stop_sec + 0.25:
            self._finish(False, 'no zero output observed before watchdog probe timeout')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WatchdogProbe()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if rclpy.ok():
            for _ in range(10):
                node._publish()
                rclpy.spin_once(node, timeout_sec=0.02)
        succeeded = node.succeeded
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if succeeded else 2)


if __name__ == '__main__':
    main()
