"""Record the two fixed home-scene poses without commanding the robot."""

from __future__ import annotations

import math
import os
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_ros

from .scene_pose_recorder_logic import ScenePose, quaternion_to_yaw, update_scene_yaml
from .route_points_logic import (
    STATIONS, saved_map_binding, loaded_map_binding, combine_map_binding,
    read_route, record_route, route_summary,
)


class ScenePoseRecorder(Node):
    def __init__(self) -> None:
        super().__init__('scene_pose_recorder')
        self.declare_parameter(
            'output_path',
            '/home/ubuntu/ros2_ws/src/raicom_control/config/scene_home_v1.yaml',
        )
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_max_age_sec', 1.0)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('velocity_topic', '/controller/cmd_vel')
        self.declare_parameter('velocity_max_linear_mps', 0.01)
        self.declare_parameter('velocity_max_angular_rps', 0.02)
        self.declare_parameter('stationary_hold_sec', 0.5)
        self.declare_parameter('route_output_path', '/home/ubuntu/ros2_ws/src/raicom_control/config/route_map_01.json')
        self.declare_parameter('route_map_file', '/home/ubuntu/ros2_ws/src/slam/maps/map_01.yaml')
        self._loaded_map = None
        self._route_saved_map = None
        self._lock = threading.Lock()
        self._pose: ScenePose | None = None
        self._pose_at: float | None = None
        self._pose_frame = ''
        self._velocity_at: float | None = None
        self._stationary_since: float | None = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        pose_topic = str(self.get_parameter('pose_topic').value)
        velocity_topic = str(self.get_parameter('velocity_topic').value)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self._on_pose, 10)
        self.create_subscription(Twist, velocity_topic, self._on_velocity, 10)
        self._status_pub = self.create_publisher(String, '/raicom/scene/pose_status', 10)
        self.create_service(Trigger, '/raicom/scene/record_pick', self._record_pick)
        self.create_service(Trigger, '/raicom/scene/record_place', self._record_place)
        self.create_service(Trigger, '/raicom/scene/clear_recorded_poses', self._clear)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for station in STATIONS:
            self.create_service(Trigger, '/raicom/route/record_' + station,
                                lambda req, res, station=station: self._record_route(station, res))
        self.create_service(Trigger, '/raicom/route/list', self._list_route)
        self._publish('IDLE: pose recorder only; no motion or servo publisher exists')

    def _publish(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)
        self.get_logger().info(text)

    def _on_map(self, message):
        try:
            self._loaded_map = loaded_map_binding(message)
        except (ValueError, TypeError, OverflowError) as exc:
            self._loaded_map = None
            self._publish('ROUTE MAP REJECTED: ' + str(exc))

    def _route_binding(self):
        publishers = self.get_publishers_info_by_topic('/map')
        if len(publishers) != 1 or publishers[0].node_name != 'map_server':
            raise RuntimeError('Route recording requires exactly one fixed map_server')
        if self._loaded_map is None:
            raise RuntimeError('Fixed map has not arrived')
        saved = saved_map_binding(str(self.get_parameter('route_map_file').value))
        if self._route_saved_map is not None and saved != self._route_saved_map:
            raise RuntimeError('Saved map changed during survey; stop and reconcile maps')
        self._route_saved_map = saved
        return combine_map_binding(saved, self._loaded_map)

    def _route_document(self, binding):
        path = Path(str(self.get_parameter('route_output_path').value)).resolve()
        legacy = Path(str(self.get_parameter('output_path').value)).resolve()
        map_file = Path(str(self.get_parameter('route_map_file').value)).resolve()
        if path.suffix != '.json' or path in (legacy, map_file):
            raise ValueError('Route output must be a separate JSON journal')
        return path, path.read_text(encoding='utf-8') if path.exists() else None

    def _record_route(self, station, response):
        try:
            binding = self._route_binding()
            pose = self._current_pose()  # Same fresh TF and continuous zero-speed gate.
            path, original = self._route_document(binding)
            updated = record_route(original, station, pose, binding)
            self._atomic_write(str(path), updated)
            response.success = True
            response.message = f'{station} recorded: x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.3f}; {path}'
            self._publish(response.message)
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
            response.success = False
            response.message = str(exc)
            self._publish('ROUTE REJECTED: ' + str(exc))
        return response

    def _list_route(self, _request, response):
        try:
            binding = self._route_binding()
            _path, original = self._route_document(binding)
            response.message = route_summary(read_route(original, binding))
            response.success = True
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != str(self.get_parameter('map_frame').value):
            return
        try:
            pose = ScenePose(
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                quaternion_to_yaw(
                    message.pose.pose.orientation.x,
                    message.pose.pose.orientation.y,
                    message.pose.pose.orientation.z,
                    message.pose.pose.orientation.w,
                ),
            )
        except ValueError as exc:
            self._publish('IGNORED: invalid map pose: ' + str(exc))
            return
        with self._lock:
            self._pose = pose
            self._pose_at = time.monotonic()
            self._pose_frame = message.header.frame_id

    def _on_velocity(self, message: Twist) -> None:
        """Track a fresh, continuously held zero command before recording."""

        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
        now = time.monotonic()
        linear_limit = float(self.get_parameter('velocity_max_linear_mps').value)
        angular_limit = float(self.get_parameter('velocity_max_angular_rps').value)
        finite = all(math.isfinite(value) for value in values)
        linear_stopped = max(abs(value) for value in values[:3]) <= linear_limit
        angular_stopped = max(abs(value) for value in values[3:]) <= angular_limit
        with self._lock:
            self._velocity_at = now
            if finite and linear_stopped and angular_stopped:
                if self._stationary_since is None:
                    self._stationary_since = now
            else:
                self._stationary_since = None

    def _current_pose(self) -> ScenePose:
        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        max_age = float(self.get_parameter('pose_max_age_sec').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise RuntimeError(f'{map_frame}->{base_frame} transform is missing') from exc
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1_000_000_000.0
        now_ros = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if stamp <= 0.0 or now_ros < stamp or now_ros - stamp > max_age:
            raise RuntimeError(f'{map_frame}->{base_frame} transform is stale')
        pose = ScenePose(
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            quaternion_to_yaw(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
        )
        now = time.monotonic()
        with self._lock:
            velocity_at, stationary_since = self._velocity_at, self._stationary_since
        if velocity_at is None or now - velocity_at > float(
            self.get_parameter('pose_max_age_sec').value
        ):
            raise RuntimeError('base velocity is missing or stale')
        if stationary_since is None or now - stationary_since < float(
            self.get_parameter('stationary_hold_sec').value
        ):
            raise RuntimeError('base has not held zero velocity long enough')
        return pose

    def _atomic_write(self, path: str, content: str) -> None:
        directory = os.path.dirname(path) or '.'
        fd, temporary = tempfile.mkstemp(prefix='.scene_home_v1.', suffix='.tmp', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8', newline='') as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _record(self, station: str, response: Trigger.Response) -> Trigger.Response:
        try:
            pose = self._current_pose()
            with self._lock:
                path = os.path.abspath(str(self.get_parameter('output_path').value))
                with open(path, 'r', encoding='utf-8', newline='') as stream:
                    original = stream.read()
                updated = update_scene_yaml(original, station, pose)
                if updated == original:
                    raise RuntimeError('recorded pose did not change configuration')
                self._atomic_write(path, updated)
            text = f'{station} recorded: x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.3f}'
            self._publish(text)
            response.success = True
            response.message = text
        except (OSError, RuntimeError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self._publish('REJECTED: ' + response.message)
        return response

    def _record_pick(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._record('pick', response)

    def _record_place(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._record('place', response)

    def _clear(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            with self._lock:
                path = os.path.abspath(str(self.get_parameter('output_path').value))
                with open(path, 'r', encoding='utf-8', newline='') as stream:
                    original = stream.read()
                cleared = update_scene_yaml(original, 'pick', ScenePose(0.0, 0.0, 0.0))
                cleared = update_scene_yaml(cleared, 'place', ScenePose(0.0, 0.0, 0.0))
                cleared = cleared.replace('    goal_configured: true', '    goal_configured: false', 1)
                cleared = cleared.replace('    place_configured: true', '    place_configured: false', 1)
                self._atomic_write(path, cleared)
            response.success = True
            response.message = 'recorded pick/place flags cleared'
            self._publish('IDLE: ' + response.message)
        except (OSError, RuntimeError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            self._publish('REJECTED: ' + response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenePoseRecorder()
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
