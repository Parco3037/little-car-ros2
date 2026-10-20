"""On-demand bridge from the existing YOLO web API to Aurora ROI input.

The bridge does not start or rebuild a container and does not publish any
motion command.  A caller explicitly invokes ``/raicom/vision/refresh``; the
bridge asks the already-running web service for one inference, rejects zero or
ambiguous targets, and publishes one normalized semantic ROI for Aurora.
"""

from __future__ import annotations

import json
import threading
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .yolo_roi_logic import (
    Detection,
    choose_detection,
    detections_in_normalized_roi,
    normalized_roi,
)


class YoloRoiBridge(Node):
    def __init__(self) -> None:
        super().__init__('yolo_roi_bridge')
        self.declare_parameter('api_url', 'http://127.0.0.1:8080/api/do_infer')
        self.declare_parameter('desired_label', '')
        self.declare_parameter('candidate_roi_json', '')
        self.declare_parameter('minimum_score', 0.4)
        self.declare_parameter('timeout_sec', 8.0)
        self._roi_pub = self.create_publisher(String, '/raicom/vision/target_roi', 10)
        self._status_pub = self.create_publisher(String, '/raicom/vision/status', 10)
        self._refresh_lock = threading.Lock()
        self.create_service(Trigger, '/raicom/vision/refresh', self._on_refresh)

    def _on_refresh(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._refresh_lock.acquire(blocking=False):
            response.success = False
            response.message = 'YOLO refresh already in progress'
            return response
        threading.Thread(target=self._refresh_worker, daemon=True).start()
        response.success = True
        response.message = 'YOLO refresh queued; observe /raicom/vision/status'
        return response

    def _refresh_worker(self) -> None:
        try:
            request = Request(str(self.get_parameter('api_url').value), method='POST')
            with urlopen(request, timeout=float(self.get_parameter('timeout_sec').value)) as stream:
                payload = json.loads(stream.read().decode('utf-8'))
            if payload.get('status') != 'success':
                raise RuntimeError(str(payload.get('error', 'YOLO inference failed')))
            detections = []
            for item in payload.get('detections', []):
                bbox = item.get('bbox')
                if not isinstance(bbox, list) or len(bbox) != 4:
                    continue
                detections.append(Detection(str(item.get('class', '')), float(item.get('score', 0.0)), tuple(map(float, bbox))))
            width, height = map(int, payload.get('image_size', [0, 0]))
            candidate_roi = self._candidate_roi()
            if candidate_roi is not None:
                detections = detections_in_normalized_roi(
                    detections,
                    candidate_roi,
                    image_width=width,
                    image_height=height,
                )
            target = choose_detection(
                detections,
                desired_label=str(self.get_parameter('desired_label').value),
                minimum_score=float(self.get_parameter('minimum_score').value),
            )
            if target is None:
                raise RuntimeError('no unique qualifying YOLO target')
            roi = normalized_roi(target, width, height)
            msg = String()
            msg.data = json.dumps(roi, separators=(',', ':'))
            self._roi_pub.publish(msg)
            self._publish_status('ROI published for ' + target.label)
        except Exception as exc:
            self._publish_status('ROI rejected: ' + str(exc))
        finally:
            self._refresh_lock.release()

    def _candidate_roi(self) -> tuple[float, float, float, float] | None:
        raw = str(self.get_parameter('candidate_roi_json').value).strip()
        if not raw:
            return None
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise ValueError('candidate_roi_json must be an object')
        return (
            float(payload['x']),
            float(payload['y']),
            float(payload['w']),
            float(payload['h']),
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloRoiBridge()
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
