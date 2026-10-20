"""Publish a fixed-size LaserScan from the variable MS200 stream."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


TAU = 2.0 * math.pi


class ScanNormalizer(Node):
    """Bin variable-length scans into a stable angular grid for SLAM/Nav2."""

    def __init__(self) -> None:
        super().__init__('scan_normalizer')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan_stable')
        self.declare_parameter('samples', 254)
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', TAU)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.samples = int(self.get_parameter('samples').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        if self.samples < 32:
            raise ValueError('samples must be at least 32')
        if not math.isfinite(self.angle_min) or not math.isfinite(self.angle_max):
            raise ValueError('angle_min and angle_max must be finite')
        if not self.angle_min < self.angle_max <= self.angle_min + TAU + 1e-6:
            raise ValueError('angle range must be positive and no wider than 2*pi')

        self.target_increment = (self.angle_max - self.angle_min) / (self.samples - 1)
        self.publisher = self.create_publisher(
            LaserScan, self.output_topic, self._reliable_qos()
        )
        self.subscription = self.create_subscription(
            LaserScan, self.input_topic, self._on_scan, self._reliable_qos()
        )
        self._last_input_count: int | None = None
        self._last_change_log_at = 0.0
        self.get_logger().info(
            f'normalizing {self.input_topic} -> {self.output_topic}; '
            f'samples={self.samples}, angles={self.angle_min:.4f}..{self.angle_max:.4f}'
        )

    @staticmethod
    def _reliable_qos() -> QoSProfile:
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _on_scan(self, message: LaserScan) -> None:
        source_count = len(message.ranges)
        if source_count < 2:
            self.get_logger().warn('dropping LaserScan with fewer than 2 range readings')
            return
        if source_count != self._last_input_count:
            now = time.monotonic()
            if self._last_input_count is not None and now - self._last_change_log_at >= 5.0:
                self.get_logger().warn(
                    f'input ranges changed {self._last_input_count} -> {source_count}; '
                    f'publishing fixed {self.samples}-sample scan'
                )
                self._last_change_log_at = now
            self._last_input_count = source_count

        source_increment = float(message.angle_increment)
        if not math.isfinite(source_increment) or source_increment <= 0.0:
            source_increment = (
                float(message.angle_max) - float(message.angle_min)
            ) / max(source_count - 1, 1)
        if not math.isfinite(source_increment) or source_increment <= 0.0:
            self.get_logger().warn('dropping LaserScan with invalid angular metadata')
            return

        # MS200 uses zero (and occasionally values outside the advertised
        # limits) for an invalid return.  Passing those values through makes
        # SLAM treat the sensor origin as an obstacle and produces filled or
        # jagged maps.  Keep invalid bins as NaN so LaserScan consumers ignore
        # them while retaining real returns within the sensor limits.
        range_min = float(message.range_min)
        range_max = float(message.range_max)
        if not math.isfinite(range_min) or range_min < 0.0:
            range_min = 0.0
        if not math.isfinite(range_max) or range_max <= range_min:
            range_max = math.inf

        ranges = [float('nan')] * self.samples
        for index, raw_value in enumerate(message.ranges):
            value = float(raw_value)
            if not math.isfinite(value) or value < range_min or value > range_max:
                continue
            angle = float(message.angle_min) + index * source_increment
            while angle < self.angle_min:
                angle += TAU
            while angle > self.angle_max:
                angle -= TAU
            bin_index = int(round((angle - self.angle_min) / self.target_increment))
            if not 0 <= bin_index < self.samples:
                continue
            current = ranges[bin_index]
            if not math.isfinite(current) or value < current:
                ranges[bin_index] = value

        output = LaserScan()
        output.header = message.header
        output.angle_min = self.angle_min
        output.angle_max = self.angle_max
        output.angle_increment = self.target_increment
        output.time_increment = (
            float(message.scan_time) / max(self.samples - 1, 1)
            if math.isfinite(message.scan_time) and message.scan_time > 0.0
            else 0.0
        )
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        output.ranges = ranges
        output.intensities = []
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
