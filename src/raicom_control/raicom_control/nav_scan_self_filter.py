"""Sensor-only node owned by field_tool_navigation; never sends motion commands."""
from copy import deepcopy
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .nav_scan_geometry import NAV_SCAN_TOPIC, footprint_points, mask_body_returns


class NavigationScanSelfFilter(Node):
    def __init__(self, **kwargs):
        super().__init__('nav_scan_self_filter', **kwargs)
        self.declare_parameter('footprint', '')
        self.footprint = footprint_points(self.get_parameter('footprint').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LaserScan, NAV_SCAN_TOPIC, qos_profile_sensor_data)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.received = self.removed = 0
        self.get_logger().info('Navigation body mask: footprint plus 0.005 m; farther returns retained')

    def on_scan(self, message):
        try:
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            if age > 0.5 or age < -0.1:
                raise ValueError('stale/future scan')
            transform = self.buffer.lookup_transform('base_footprint', message.header.frame_id, stamp).transform
            t, q = transform.translation, transform.rotation
            values, removed = mask_body_returns(
                message.ranges, message.angle_min, message.angle_increment,
                message.range_min, message.range_max,
                (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), self.footprint)
        except Exception as exc:
            # No fabricated free scan on TF/input failure. Costmap observation
            # timeout then prevents using stale data for continued navigation.
            self.get_logger().warning('Navigation scan unavailable: ' + str(exc), throttle_duration_sec=5.)
            return
        output = deepcopy(message)
        output.ranges = values
        self.publisher.publish(output)
        self.received += 1
        self.removed += len(removed)
        if removed:
            self.get_logger().info(json.dumps({'body_return_indices': removed,
                'removed_total': self.removed, 'frames': self.received}), throttle_duration_sec=5.)


def main():
    rclpy.init()
    node = NavigationScanSelfFilter()
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
