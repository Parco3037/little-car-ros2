"""ROS-facing state for the field mapping and navigation tool.

The class publishes only guarded teleop key events, motion-enable leases,
manual suppression, initial-pose estimates, and navigation goal contexts.
It never publishes Twist directly.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from .scene_pose_recorder_logic import quaternion_to_yaw
from .field_tool_navigation_logic import NavigationController, NavigationTarget


class FieldToolNode(Node):
    def __init__(self) -> None:
        super().__init__('raicom_field_tool')
        self.declare_parameter('map_dir', '/home/ubuntu/raicom_tools/field_tool_data/maps')
        self.declare_parameter('route_dir', '/home/ubuntu/raicom_tools/field_tool_data/routes')
        self.declare_parameter('lease_ttl_s', 0.25)
        self.declare_parameter('zero_hold_s', 0.5)
        self.declare_parameter('manual_linear_speed', 0.25)
        self.declare_parameter('manual_angular_speed', 0.50)
        self.declare_parameter('start_mapping', False)
        self.manual_linear_speed = float(self.get_parameter('manual_linear_speed').value)
        self.manual_angular_speed = float(self.get_parameter('manual_angular_speed').value)
        if not 0.05 <= self.manual_linear_speed <= 0.50:
            raise ValueError('manual_linear_speed must be 0.05..0.50 m/s')
        if not 0.05 <= self.manual_angular_speed <= 0.50:
            raise ValueError('manual_angular_speed must be 0.05..0.50 rad/s')

        self.map_dir = Path(str(self.get_parameter('map_dir').value)).resolve()
        self.route_dir = Path(str(self.get_parameter('route_dir').value)).resolve()
        self.lease_ttl = float(self.get_parameter('lease_ttl_s').value)
        self.zero_hold_s = float(self.get_parameter('zero_hold_s').value)
        if not 0.1 <= self.lease_ttl <= 0.5:
            raise ValueError('lease TTL must be 0.1..0.5 seconds')
        if not 0.1 <= self.zero_hold_s <= 5.0:
            raise ValueError('zero hold must be 0.1..5 seconds')

        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_message: OccupancyGrid | None = None
        self.amcl_pose = None
        self.scan_at: float | None = None
        self.scan_stamp = None
        self.velocity_at: float | None = None
        self.velocity_zero = True
        self.velocity_zero_since: float | None = None
        self.navigation_active = False
        self.navigation_active_at = 0.0
        self.navigation_active_subscription = self.create_subscription(
            Bool, '/raicom/navigation_active', self._on_navigation_active, latched)

        self.key_publisher = self.create_publisher(String, '/raicom/teleop_key', 10)
        self.lease_publisher = self.create_publisher(String, '/raicom/motion_enable', 1)
        self.suppress_publisher = self.create_publisher(
            Bool, '/raicom/manual_suppressed', 10)
        self.context_publisher = self.create_publisher(
            String, '/raicom/nav_goal_context', 1)
        self.stop_publisher = self.create_publisher(Empty, '/raicom/nav_stop', 10)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.navigation_state_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('controller_server', 'planner_server', 'smoother_server',
                         'behavior_server', 'bt_navigator', 'waypoint_follower',
                         'velocity_smoother')
        }
        self.navigation_state_requests = {}
        self.localization_start_clients = {
            'map_server': self.create_client(GetState, '/map_server/get_state'),
            'amcl': self.create_client(GetState, '/amcl/get_state'),
            'map_file': self.create_client(GetParameters, '/map_server/get_parameters'),
        }
        self.localization_start_requests = {}
        self.controller_parameters_client = self.create_client(
            SetParameters, '/controller_server/set_parameters')
        self.navigation = NavigationController(self)
        from .field_tool_chain import CommandChainObserver
        self.command_chain = CommandChainObserver(self)
        self.external_stop_requested = False
        self.external_stop_generation = 0
        self.external_stop_subscription = self.create_subscription(
            Empty, '/raicom/nav_stop', self._on_external_stop, 10)
        self._motion_mission_id = ''
        self.correction_deadline = 0.0
        self.correction_started_ns = 0
        self.correction_context = ''
        self.correction_status = ''
        self.correction_lock = threading.RLock()
        self.candidate_subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/raicom/initialpose_candidate',
            self._on_initialpose_candidate, 1)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/raicom/field_points', latched)
        self.ui_snapshot = {}
        self.control_parameters = {}
        self._parameter_pending = {}
        self._speed_clients = {
            name: self.create_client(GetParameters, '/' + name + '/get_parameters')
            for name in ('keyboard_input', 'cmd_vel_guard')}
        from .field_tool_sensors import FieldToolSensors
        self.sensors = FieldToolSensors(self)
        self.tf_buffer = self.sensors.buffer
        self.sensors.start()
        self._health_stop = threading.Event()
        self._health_thread = threading.Thread(
            target=self._health_loop, name='field-health-cache', daemon=True)
        self._health_thread.start()
        # Preview/action callbacks may block the general executor. Autonomous
        # renewal must run independently, using the separate sensor executor.
        from .field_tool_nav_heartbeat import NavigationHeartbeatWorker
        self.navigation_heartbeat_worker = NavigationHeartbeatWorker(
            lambda: self.navigation.heartbeat, self.get_logger().error)
        self.navigation_heartbeat_worker.start()

    def _health_loop(self):
        """TF waits and discovery never run on the Tk input/renewal loop."""
        last_parameters = 0.0
        while not self._health_stop.is_set():
            started = time.monotonic()
            state = {'at': started, 'base_ok': False, 'fixed_ok': False,
                     'scan_ok': False, 'pose': None}
            try:
                state['base_ok'] = self.base_tf_ok()
                state['fixed_ok'] = self.fixed_tf_ok()
                state['scan_ok'] = self.sensor_ready()
                if not state['base_ok'] or not state['scan_ok']:
                    details = self.base_tf_details()
                    age = details['odom_base_age_s']
                    if details['odom_base_error']:
                        state['sensor_error'] = '底盘坐标未收到：' + details['odom_base_error']
                    elif age is not None and not 0 <= age <= .5:
                        state['sensor_error'] = f'底盘定位时间异常：{age:.3f}s（要求0–0.500s）'
                    elif not details['base_link_exists'] or not details['lidar_exists']:
                        state['sensor_error'] = '底盘或雷达固定坐标链缺失'
                    else:
                        state['sensor_error'] = '激光数据未收到或已过期（要求≤0.500s）'
                    if started - getattr(self, '_sensor_fault_at', 0.) >= 2.:
                        self._sensor_fault_at = started
                        self.get_logger().warning('FIELD_SENSOR_FAULT ' + json.dumps(dict(
                            **details, reason=state['sensor_error'], scan_received_age_s=(
                                None if self.scan_at is None else time.monotonic()-self.scan_at))))
                owners = {
                    topic: sorted('/' + '/'.join(filter(None, (
                        info.node_namespace.strip('/'), info.node_name)))
                        for info in self.get_publishers_info_by_topic(topic))
                    for topic in ('/raicom/teleop_cmd', '/raicom/teleop_key',
                                  '/controller/cmd_vel', '/raicom/navigation_active')}
                state['control_publishers'] = owners
                state['controls_unique'] = self.manual_control_owners_ok(owners)
                state['nav_bridge_present'] = '/nav_cmd_bridge' in owners['/raicom/teleop_cmd']
                if state['fixed_ok']:
                    tf, age = self.fixed_map_transform()
                    p, q = tf.transform.translation, tf.transform.rotation
                    state['pose'] = (p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w), age)
                if started - last_parameters >= 1.0:
                    last_parameters = started
                    self._request_control_parameters()
            except Exception as exc:
                state['error'] = str(exc)
            self.ui_snapshot = state
            self._health_stop.wait(.2)

    def _request_control_parameters(self):
        names = {'keyboard_input': ['linear_speed', 'angular_speed'],
                 'cmd_vel_guard': ['max_linear', 'max_angular']}
        for node, client in self._speed_clients.items():
            pending = self._parameter_pending.get(node)
            if pending and not pending[0].done():
                if time.monotonic() - pending[1] < 2.0:
                    continue
                client.remove_pending_request(pending[0])
                pending[0].cancel()
            if not client.service_is_ready():
                continue
            request = GetParameters.Request(); request.names = names[node]
            future = client.call_async(request)
            self._parameter_pending[node] = (future, time.monotonic())
            def received(f, name=node):
                try:
                    values = f.result().values
                    if len(values) == 2 and all(v.type == 3 for v in values):
                        self.control_parameters[name] = (
                            values[0].double_value, values[1].double_value,
                            time.monotonic())
                except Exception:
                    pass
            future.add_done_callback(received)

    @staticmethod
    def manual_control_owners_ok(owners):
        # The original keyboard and navigation adapters coexist while idle.
        # Duplicate or unrelated owners are still rejected.
        teleop = sorted(owners.get('/raicom/teleop_cmd', []))
        active = sorted(owners.get('/raicom/navigation_active', []))
        return (
            teleop in (['/keyboard_input'], ['/keyboard_input', '/nav_cmd_bridge'])
            and owners.get('/raicom/teleop_key') == ['/raicom_field_tool']
            and owners.get('/controller/cmd_vel') == ['/cmd_vel_guard']
            and active == (['/nav_cmd_bridge'] if '/nav_cmd_bridge' in teleop else []))

    def _on_navigation_active(self, message):
        self.navigation_active = bool(message.data)
        self.navigation_active_at = time.monotonic()

    def manual_health_error(self):
        state = self.ui_snapshot
        now = time.monotonic()
        k = self.control_parameters.get('keyboard_input')
        g = self.control_parameters.get('cmd_vel_guard')
        if now - state.get('at', 0.) >= .6:
            return '状态更新超时'
        if not state.get('base_ok') or not state.get('scan_ok'):
            return state.get('sensor_error') or '等待底盘 TF 和激光'
        if not state.get('controls_unique'):
            return '控制来源缺失或重复'
        if state.get('nav_bridge_present'):
            if now - self.navigation_active_at >= .5:
                return '等待导航桥停止确认'
            if self.navigation_active:
                return '导航仍在执行'
        if not k or not g or now - k[2] >= 3.0 or now - g[2] >= 3.0:
            return '等待实际速度参数'
        if not (abs(k[0] - self.manual_linear_speed) < 1e-6
                and abs(k[1] - self.manual_angular_speed) < 1e-6
                and k[0] <= g[0] <= 2.0 and k[1] <= g[1] <= 2.0):
            return '手动速度与守护上限不一致'
        return ''

    def manual_health_ok(self):
        return not FieldToolNode.manual_health_error(self)

    def arm_initialpose(self, context):
        if (not context or not self.map_message or not self.base_tf_ok()
                or not self.sensor_ready() or not self.zero_velocity_hold_ok()
                or not self.initial_pose_subscriber_ready()):
            raise RuntimeError('纠偏需要已加载地图、定位服务、正常传感器及停车')
        with self.correction_lock:
            self.correction_context = str(context)
            self.correction_started_ns = self.get_clock().now().nanoseconds
            self.correction_deadline = time.monotonic() + 30.0
            self.correction_status = '请在 RViz 用 2D Pose Estimate 标出车位和朝向（30秒内）'

    def cancel_initialpose(self):
        with self.correction_lock:
            self.correction_deadline = 0.0
            self.correction_context = ''

    def _on_initialpose_candidate(self, message):
        with self.correction_lock:
            if not self.correction_context or time.monotonic() > self.correction_deadline:
                return
            try:
                stamp = Time.from_msg(message.header.stamp).nanoseconds
                age = (self.get_clock().now().nanoseconds - stamp) / 1e9
                p, q = message.pose.pose.position, message.pose.pose.orientation
                values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                if (message.header.frame_id != 'map' or stamp < self.correction_started_ns
                        or not 0.0 <= age <= 2.0
                        or not all(math.isfinite(v) for v in values)
                        or abs(sum(v*v for v in (q.x,q.y,q.z,q.w)) - 1.) > .001
                        or abs(q.x) > .001 or abs(q.y) > .001):
                    raise RuntimeError('RViz 位姿无效、过期或坐标系不符')
                self.publish_initial_pose(p.x, p.y, quaternion_to_yaw(q.x,q.y,q.z,q.w))
                self.cancel_initialpose()
                self.correction_status = 'RViz 初始位姿已通过检查并发布'
            except Exception as exc:
                self.correction_status = '纠偏未执行：' + str(exc)

    def publish_points(self, document=None, drafts=None):
        array = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; array.markers.append(clear)
        points = []
        if document:
            for number, name in enumerate(document.get('order', []), 1):
                point = document['points'][name]
                for kind in ('reference', 'approach'):
                    pose = point.get(kind)
                    if pose:
                        points.append((f'{number} {point["label"]} {kind}', pose, kind == 'reference'))
        for p in drafts or []:
            points.append(('draft '+p['name'], p, True))
        for index, (label, pose, reference) in enumerate(points):
            marker = Marker(); marker.header.frame_id = 'map'
            marker.ns = 'field_points'; marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING; marker.action = Marker.ADD
            marker.pose.position.x = float(pose['x']); marker.pose.position.y = float(pose['y'])
            marker.pose.position.z = .10; marker.pose.orientation.w = 1.0
            marker.scale.z = .06; marker.color.a = 1.0
            marker.color.r = 1.0 if reference else .1
            marker.color.g = .8; marker.color.b = .2 if reference else 1.0
            marker.text = label; array.markers.append(marker)
        self.marker_publisher.publish(array)

    def stop_health_worker(self):
        self.navigation_heartbeat_worker.close()
        self._health_stop.set()
        self._health_thread.join(timeout=2.0)
        self.sensors.close()

    def _on_external_stop(self, message):
        self.navigation.heartbeat.on_external_stop()

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _on_amcl(self, message) -> None:
        self.amcl_pose = message

    def _on_scan(self, message) -> None:
        self.scan_stamp = message.header.stamp
        self.scan_at = time.monotonic()
        self.scan_observation = (message.header.stamp, self.scan_at)

    def _on_velocity(self, message: Twist) -> None:
        self.velocity_at = time.monotonic()
        self.velocity_zero = all(value == 0.0 for value in (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ))
        if self.velocity_zero:
            if self.velocity_zero_since is None:
                self.velocity_zero_since = self.velocity_at
        else:
            self.velocity_zero_since = None

    def set_motion_mission(self, mission_id: str) -> None:
        self._motion_mission_id = mission_id

    def publish_lease(self, enabled: bool, mission_id: str | None = None) -> None:
        payload = {
            'boot_id': self.boot_id,
            'mission_id': mission_id or self._motion_mission_id,
            'enabled': bool(enabled),
            'expires_at': time.monotonic() + (
                self.lease_ttl if enabled else 0.0),
        }
        self.lease_publisher.publish(String(
            data=json.dumps(payload, separators=(',', ':'))))

    def publish_manual_suppressed(self, active: bool) -> None:
        self.suppress_publisher.publish(Bool(data=bool(active)))

    def publish_goal_context(
            self, goal_id: str, *, active: bool, mission_id: str) -> None:
        payload = {
            'boot_id': self.boot_id,
            'mission_id': mission_id,
            'goal_id': goal_id,
            'active': bool(active),
            'expires_at': time.monotonic() + (0.5 if active else 0.0),
        }
        self.context_publisher.publish(String(
            data=json.dumps(payload, separators=(',', ':'))))

    def publish_nav_stop(self) -> None:
        for _ in range(3):
            self.stop_publisher.publish(Empty())

    def send_key(self, key: str) -> None:
        self.key_publisher.publish(String(data=key))

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        if (not all(math.isfinite(float(v)) for v in (x, y, yaw))
                or self.map_message is None or not self.base_tf_ok()
                or not self.sensor_ready() or not self.zero_velocity_hold_ok()
                or not self.initial_pose_subscriber_ready()):
            raise RuntimeError('initial pose requires a loaded map, fresh sensors, AMCL and stopped robot')
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        base_transform = self.tf_buffer.lookup_transform(
            'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.05))
        message.header.stamp = base_transform.header.stamp
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        half = float(yaw) / 2.0
        message.pose.pose.orientation.z = math.sin(half)
        message.pose.pose.orientation.w = math.cos(half)
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.25
        message.pose.covariance[35] = 0.06853891909122467
        self.initial_pose_publisher.publish(message)

    def initial_pose_subscriber_ready(self) -> bool:
        try:
            return bool(
                self.get_subscriptions_info_by_topic('/initialpose'))
        except Exception:
            return False

    def reset_navigation_readiness(self):
        self.navigation_active_since = None
        self.navigation_state_samples = {}
        self.navigation_readiness_status = {}
        for name, (future, _) in self.navigation_state_requests.items():
            self.navigation_state_clients[name].remove_pending_request(future)
            future.cancel()
        self.navigation_state_requests.clear()

    def navigation_ready(self):
        """Poll fresh lifecycle replies; a discoverable action can still reject goals."""
        now = time.monotonic()
        ready = self.navigate_client.server_is_ready()
        if not hasattr(self, 'navigation_state_samples'):
            self.navigation_state_samples = {}
        status = {} if ready else {'navigate_to_pose': 'Action未就绪'}
        for name, client in self.navigation_state_clients.items():
            pending = self.navigation_state_requests.get(name)
            if pending is not None:
                future, requested_at = pending
                if now - requested_at <= 1.0 and future.done():
                    try:
                        response = future.result()
                    except Exception:
                        response = None
                    if response is not None:
                        self.navigation_state_samples[name] = (response.current_state.id, requested_at)
                    else:
                        self.navigation_state_samples.pop(name, None)
                    client.remove_pending_request(future)
                    del self.navigation_state_requests[name]
                elif now - requested_at > 1.0:
                    client.remove_pending_request(future)
                    future.cancel()
                    del self.navigation_state_requests[name]
                    self.navigation_state_samples.pop(name, None)
            sample = self.navigation_state_samples.get(name)
            available = client.service_is_ready()
            if not available:
                self.navigation_state_samples.pop(name, None)
                sample = None
            fresh = sample is not None and now - sample[1] <= 1.0
            active = available and fresh and sample[0] == 3
            if not active:
                ready = False
                status[name] = ('服务未发现' if not available else
                                '状态等待/过期' if not fresh else
                                {1: '未配置', 2: 'Inactive', 4: '已结束'}.get(sample[0], '状态切换中'))
            # Refresh BEFORE expiry, keeping the last fresh observation while
            # its replacement is pending. Seven staggered requests otherwise
            # continually reset the all-Active settling window on a cold start.
            if (available and name not in self.navigation_state_requests
                    and (not active or now - sample[1] >= .4)):
                self.navigation_state_requests[name] = (
                    client.call_async(GetState.Request()), now)
        self.navigation_readiness_status = status
        if not ready:
            self.navigation_active_since = None
            return False
        if getattr(self, 'navigation_active_since', None) is None:
            self.navigation_active_since = now
        # All nodes must remain freshly Active for .35 s. Discovery alone
        # is insufficient, and resetting an attempt discards every observation.
        return now - self.navigation_active_since >= .35

    def navigation_readiness_detail(self):
        status = getattr(self, 'navigation_readiness_status', {})
        return '；'.join(f'{name}: {reason}' for name, reason in status.items()) or '节点已Active，等待稳定/参数回读'

    def set_controller_tolerances(
            self, position_m: float, yaw_deg: float):
        if not self.controller_parameters_client.service_is_ready():
            return
        request = SetParameters.Request()
        for name, value in (
                ('general_goal_checker.xy_goal_tolerance', float(position_m)),
                ('general_goal_checker.yaw_goal_tolerance',
                 math.radians(float(yaw_deg))),
                ('FollowPath.xy_goal_tolerance', float(position_m))):
            parameter = ParameterMessage()
            parameter.name = name
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value))
            request.parameters.append(parameter)
        parameter = ParameterMessage()
        parameter.name = 'general_goal_checker.stateful'
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=False)
        request.parameters.append(parameter)
        return self.controller_parameters_client.call_async(request)

    def record_navigation_arrival(self, report):
        line = json.dumps(report, ensure_ascii=False, allow_nan=False)
        self.get_logger().info('NAVIGATION_ARRIVAL ' + line)
        folder = Path('/home/ubuntu/raicom_runtime/field_tool_sessions')
        with (folder / 'navigation_arrivals.ndjson').open('a', encoding='utf-8') as output:
            output.write(line + '\n')

    def make_navigation_goal(self, target):
        from .field_tool_recovery import materialize_recovery_tree
        goal = NavigateToPose.Goal()
        goal.behavior_tree = materialize_recovery_tree(self.navigation.active_max_recoveries)
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(target.x)
        message.pose.position.y = float(target.y)
        message.pose.orientation.z = math.sin(float(target.yaw) / 2.0)
        message.pose.orientation.w = math.cos(float(target.yaw) / 2.0)
        goal.pose = message
        return goal

    def fixed_map_transform(self, max_age_s=0.5):
        # Time() selects the latest COMMON time along the whole chain. SLAM
        # can trail odometry; AMCL can postdate map->odom. Never extrapolate to
        # the latest odom stamp, or compose independently timed edge values.
        # Zero timeouts keep capture/save callbacks from sleeping on the Tk loop.
        base = self.tf_buffer.lookup_transform(
            'odom', 'base_footprint', Time(), timeout=Duration())
        transform = self.tf_buffer.lookup_transform(
            'map', 'base_footprint', Time(), timeout=Duration())
        now_ns = self.get_clock().now().nanoseconds
        base_age = (now_ns - Time.from_msg(base.header.stamp).nanoseconds) / 1e9
        common_age = (now_ns - Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
        if not 0.0 <= base_age <= max_age_s:
            raise RuntimeError(f'odometry TF is stale or future dated：时差{base_age:.3f}s，要求0–{max_age_s:.3f}s')
        if not 0.0 <= common_age <= max_age_s:
            raise RuntimeError(f'map TF common time is stale or future dated：时差{common_age:.3f}s，要求0–{max_age_s:.3f}s')
        return transform, max(base_age, common_age)

    def reset_map_observation(self):
        self.cancel_initialpose()
        for name, (future, _) in self.localization_start_requests.items():
            self.localization_start_clients[name].remove_pending_request(future)
            future.cancel()
        self.localization_start_requests.clear()
        self.map_message = None
        self.amcl_pose = None
        self.tf_buffer.clear()  # Local observations only; does not publish or alter TF.

    def localization_start_ready(self, expected_map: str) -> bool:
        """Fresh active states and exact map identity before automatic seeding."""
        if (self.map_message is None or not self.sensor_ready()
                or not self.zero_velocity_hold_ok()):
            return False
        publishers = self.get_publishers_info_by_topic('/map')
        if len(publishers) != 1 or publishers[0].node_name != 'map_server':
            return False
        now = time.monotonic()
        ready = True
        for name, client in self.localization_start_clients.items():
            pending = self.localization_start_requests.get(name)
            if pending is not None:
                future, requested = pending
                if now - requested <= 1.0:
                    if not future.done():
                        ready = False
                        continue
                    try:
                        response = future.result()
                        if name == 'map_file':
                            valid = (len(response.values) == 1
                                     and response.values[0].string_value == expected_map)
                        else:
                            valid = response.current_state.id == 3
                        ready = ready and valid
                    except Exception:
                        ready = False
                    continue
                client.remove_pending_request(future)
                future.cancel()
            ready = False
            if client.service_is_ready():
                request = GetParameters.Request() if name == 'map_file' else GetState.Request()
                if name == 'map_file':
                    request.names = ['yaml_filename']
                self.localization_start_requests[name] = (client.call_async(request), now)
        return ready

    def lookup_pose(
            self, target_frame: str, source_frame: str,
            timeout_s: float = 0.5) -> tuple[float, float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=timeout_s),
        )
        stamp = (
            transform.header.stamp.sec
            + transform.header.stamp.nanosec / 1_000_000_000.0
        )
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        age = now - stamp
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(
            rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw, age

    def transform_exists(
            self, target_frame: str, source_frame: str,
            timeout_s: float = 0.05) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout_s),
            )
            return True
        except Exception:
            return False

    def base_tf_ok(self, max_age_s: float = 0.5) -> bool:
        try:
            _x, _y, _yaw, age = self.lookup_pose(
                'odom', 'base_footprint', timeout_s=0.05)
            if age < 0.0 or age > max_age_s:
                return False
            return (
                self.transform_exists('base_footprint', 'base_link')
                and self.transform_exists('base_link', 'lidar_frame')
            )
        except Exception:
            return False

    def base_tf_details(self) -> dict:
        details = {
            'odom_base_age_s': None,
            'odom_base_error': None,
            'base_link_exists': False,
            'lidar_exists': False,
        }
        try:
            _x, _y, _yaw, age = self.lookup_pose(
                'odom', 'base_footprint', timeout_s=0.05)
            details['odom_base_age_s'] = age
        except Exception as exc:
            details['odom_base_error'] = repr(exc)
        details['base_link_exists'] = self.transform_exists(
            'base_footprint', 'base_link', timeout_s=0.05)
        details['lidar_exists'] = self.transform_exists(
            'base_link', 'lidar_frame', timeout_s=0.05)
        return details

    def fixed_tf_ok(self, max_age_s: float = 0.5) -> bool:
        if self.map_message is None:
            return False
        try:
            self.fixed_map_transform(max_age_s)
            return (
                self.transform_exists('base_footprint', 'base_link')
                and self.transform_exists('base_link', 'lidar_frame')
            )
        except Exception:
            return False

    def sensor_ready(self, max_scan_age_s: float = 0.5) -> bool:
        # Snapshot before reading clocks: a concurrent callback must not pair
        # an older 'now' with a just-arrived timestamp or half of another scan.
        stamp, received = getattr(self, 'scan_observation', (self.scan_stamp, self.scan_at))
        return (
            received is not None and stamp is not None
            and 0.0 <= (self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds) / 1e9 <= max_scan_age_s
            and 0.0 <= time.monotonic() - received <= max_scan_age_s
        )

    def navigation_input_error(self):
        if self.map_message is None:
            return '导航等待地图数据'
        try:
            self.fixed_map_transform(.5)
        except Exception as exc:
            return '定位数据未就绪：' + str(exc)
        if not (self.transform_exists('base_footprint', 'base_link', timeout_s=0.)
                and self.transform_exists('base_link', 'lidar_frame', timeout_s=0.)):
            return '底盘或雷达坐标链缺失'
        stamp, received = getattr(self, 'scan_observation', (self.scan_stamp, self.scan_at))
        if stamp is None or received is None:
            return '导航等待第一帧雷达数据'
        stamp_age = (self.get_clock().now().nanoseconds-Time.from_msg(stamp).nanoseconds)/1e9
        receive_age = time.monotonic()-received
        if not (0 <= stamp_age <= .5 and 0 <= receive_age <= .5):
            return f'雷达数据过期：采集{stamp_age:.3f}s / 接收{receive_age:.3f}s（要求≤0.500s）'
        return ''

    def record_navigation_fault(self, reason, target):
        self.get_logger().error('NAVIGATION_INPUT_FAULT ' + json.dumps(
            dict(reason=reason, target=target, boot_id=self.boot_id, stamp_unix=time.time()), ensure_ascii=False))

    def zero_velocity_hold_ok(self, hold_s: float | None = None) -> bool:
        hold_s = self.zero_hold_s if hold_s is None else float(hold_s)
        return (
            self.velocity_zero_since is not None
            and self.velocity_at is not None
            and time.monotonic() - self.velocity_at <= 0.5
            and self.velocity_zero
            and time.monotonic() - self.velocity_zero_since >= hold_s
        )

    def capture_current_pose(
            self, *, require_fixed_map: bool = True) -> dict:
        if require_fixed_map and self.map_message is None:
            raise RuntimeError('map is unavailable')
        if not (
                self.transform_exists('base_footprint', 'base_link', timeout_s=0.0)
                and self.transform_exists('base_link', 'lidar_frame', timeout_s=0.0)):
            raise RuntimeError('base/lidar TF chain is missing')
        try:
            # Validate the exact transform we return, once. A second query after
            # a successful boolean check can race the next odometry update.
            transform, age = self.fixed_map_transform(0.5)
        except Exception as exc:
            raise RuntimeError(f'TF chain is missing or stale: {exc}') from exc
        if not self.sensor_ready(0.5):
            raise RuntimeError('fresh /scan is unavailable')
        if not self.zero_velocity_hold_ok():
            raise RuntimeError('zero-velocity hold is not satisfied')
        position, rotation = transform.transform.translation, transform.transform.rotation
        x, y = position.x, position.y
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return {
            'x': x,
            'y': y,
            'yaw_rad': yaw,
            'yaw_deg': math.degrees(yaw),
            'tf_age_s': max(0.0, age),
            'source': 'map -> base_footprint with fresh scan and zero-velocity hold',
        }

    def halt_motion(self) -> None:
        """Stop owned motion without claiming an operator pressed emergency stop."""
        self.navigation.halt()
        self.publish_manual_suppressed(False)
        self.publish_lease(False)
        self.send_key('release')

    def emergency_stop(self) -> None:
        self.navigation.stop()
        self.publish_manual_suppressed(False)
        self.publish_lease(False)
        self.send_key(' ')
