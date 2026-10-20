"""Perception-only Aurora930 observer.

The node subscribes to the existing Aurora ROS topics and to a semantic ROI
from the SCI/YOLO bridge.  It publishes only observation/status messages; it
never publishes base, arm, servo, or gripper commands.  Missing or unstable
depth is reported as not-ready, which is the required input contract for the
later pick/place mission.
"""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .aurora_observer_logic import Roi, StabilityGate, observe_roi


class AuroraObserver(Node):
    def __init__(self) -> None:
        super().__init__('aurora_observer')
        self.declare_parameter('depth_topic', '/aurora/depth/image_raw')
        self.declare_parameter('fallback_depth_topic', '/ascamera/camera_publisher/depth0/image_raw')
        self.declare_parameter('roi_topic', '/raicom/vision/target_roi')
        self.declare_parameter('status_topic', '/raicom/aurora/status')
        self.declare_parameter('target_topic', '/raicom/aurora/target_xyz')
        self.declare_parameter('ready_topic', '/raicom/aurora/ready')
        self.declare_parameter('point_frame', 'depth_camera_link')
        # Mode 0 is the vendor launch default used for this recovery: depth
        # and IR are 320x200.  These are the device's depth/IR intrinsics.
        self.declare_parameter('fx', 211.314)
        self.declare_parameter('fy', 211.693)
        self.declare_parameter('cx', 159.686)
        self.declare_parameter('cy', 97.7588)
        self.declare_parameter('minimum_depth_mm', 150.0)
        self.declare_parameter('maximum_depth_mm', 4000.0)
        self.declare_parameter('required_stable_frames', 3)
        self.declare_parameter('max_pixel_delta', 3.0)
        self.declare_parameter('max_xyz_delta_m', 0.015)
        self.declare_parameter('input_timeout_sec', 0.5)

        self._depth: np.ndarray | None = None
        self._depth_stamp = 0.0
        self._roi: Roi | None = None
        self._roi_space = 'pixel'
        self._gate = StabilityGate(
            required_frames=int(self.get_parameter('required_stable_frames').value),
            max_pixel_delta=float(self.get_parameter('max_pixel_delta').value),
            max_xyz_delta_m=float(self.get_parameter('max_xyz_delta_m').value),
        )
        depth_topic = str(self.get_parameter('depth_topic').value)
        fallback_depth_topic = str(self.get_parameter('fallback_depth_topic').value)
        roi_topic = str(self.get_parameter('roi_topic').value)
        self._depth_subscriptions = [
            self.create_subscription(Image, depth_topic, self._on_depth, 10)
        ]
        if fallback_depth_topic and fallback_depth_topic != depth_topic:
            self._depth_subscriptions.append(
                self.create_subscription(Image, fallback_depth_topic, self._on_depth, 10)
            )
        self.create_subscription(String, roi_topic, self._on_roi, 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self._target_pub = self.create_publisher(
            PointStamped, str(self.get_parameter('target_topic').value), 10
        )
        self._ready_pub = self.create_publisher(
            Bool, str(self.get_parameter('ready_topic').value), 10
        )
        self.create_timer(0.05, self._evaluate)
        self._publish(False, 'waiting for Aurora depth and semantic ROI')

    def _on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding.lower() not in ('16uc1', 'mono16'):
                raise ValueError(f'unsupported encoding {msg.encoding!r}')
            values = np.frombuffer(msg.data, dtype=np.uint16)
            self._depth = values.reshape((msg.height, msg.width)).copy()
            self._depth_stamp = time.monotonic()
        except (ValueError, TypeError) as exc:
            self._depth = None
            self._gate.reset()
            self._publish(False, f'invalid depth frame: {exc}')

    def _on_roi(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('ROI JSON must be an object')
            space = str(payload.get('space', 'pixel')).lower()
            if space not in ('pixel', 'normalized'):
                raise ValueError('ROI space must be pixel or normalized')
            self._roi_space = space
            scale = 1_000_000 if space == 'normalized' else 1
            x = float(payload['x'])
            y = float(payload['y'])
            self._roi = Roi(
                int(round(x * scale)),
                int(round(y * scale)),
                int(round((x + float(payload['w'])) * scale)),
                int(round((y + float(payload['h'])) * scale)),
            )
            self._gate.reset()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._roi = None
            self._gate.reset()
            self._publish(False, f'invalid semantic ROI: {exc}')

    def _pixel_roi(self, roi: Roi, width: int, height: int) -> Roi:
        if self._roi_space != 'normalized':
            return roi
        # Normalized values are carried as integer micro-units by the String
        # bridge to avoid ambiguity with pixel coordinates.
        return Roi(
            int(round(roi.x1 / 1_000_000 * width)),
            int(round(roi.y1 / 1_000_000 * height)),
            int(round(roi.x2 / 1_000_000 * width)),
            int(round(roi.y2 / 1_000_000 * height)),
        )

    def _evaluate(self) -> None:
        if self._depth is None or self._roi is None:
            self._publish(False, 'waiting for Aurora depth and semantic ROI')
            return
        timeout = float(self.get_parameter('input_timeout_sec').value)
        if time.monotonic() - self._depth_stamp > timeout:
            self._gate.reset()
            self._publish(False, 'Aurora depth frame timeout')
            return
        h, w = self._depth.shape
        roi = self._pixel_roi(self._roi, w, h)
        obs = observe_roi(
            self._depth,
            roi,
            fx=float(self.get_parameter('fx').value),
            fy=float(self.get_parameter('fy').value),
            cx=float(self.get_parameter('cx').value),
            cy=float(self.get_parameter('cy').value),
            minimum_mm=float(self.get_parameter('minimum_depth_mm').value),
            maximum_mm=float(self.get_parameter('maximum_depth_mm').value),
        )
        stable = self._gate.push(obs)
        ready = bool(obs.ready and stable)
        if ready:
            point = PointStamped()
            point.header.frame_id = str(self.get_parameter('point_frame').value)
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = float(obs.x_m)
            point.point.y = float(obs.y_m)
            point.point.z = float(obs.z_m)
            self._target_pub.publish(point)
            reason = f'stable target ({self._gate.count}/{self._gate.required_frames})'
        else:
            reason = obs.reason if not obs.ready else f'stabilizing ({self._gate.count}/{self._gate.required_frames})'
        self._publish(ready, reason, obs)

    def _publish(self, ready: bool, reason: str, obs=None) -> None:
        ready_msg = Bool()
        ready_msg.data = bool(ready)
        self._ready_pub.publish(ready_msg)
        payload = {'ready': bool(ready), 'reason': reason}
        if obs is not None:
            payload.update({'valid_ratio': round(float(obs.valid_ratio), 4)})
            if obs.roi is not None:
                payload['roi'] = [obs.roi.x1, obs.roi.y1, obs.roi.x2, obs.roi.y2]
            if obs.x_m is not None:
                payload['xyz_m'] = [round(float(obs.x_m), 5), round(float(obs.y_m), 5), round(float(obs.z_m), 5)]
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AuroraObserver()
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
