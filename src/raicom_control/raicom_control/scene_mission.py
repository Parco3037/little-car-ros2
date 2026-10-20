"""Fail-closed fixed-scene navigation, pick and left-red-region placement."""

from __future__ import annotations

import math
import signal
import time
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from servo_controller_msgs.msg import ServoPosition, ServosPosition
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros

from .scene_mission_logic import (
    SceneStartConfig,
    parse_arm_profile,
    target_is_usable,
    validate_scene_start,
)


class Stage(Enum):
    IDLE = 'IDLE'
    NAVIGATING_PICK = 'NAVIGATING_PICK'
    STABILIZING_PICK = 'STABILIZING_PICK'
    REFRESHING_TARGET = 'REFRESHING_TARGET'
    WAITING_TARGET = 'WAITING_TARGET'
    ARM_PREGRASP = 'ARM_PREGRASP'
    ARM_DESCEND = 'ARM_DESCEND'
    ARM_GRASP = 'ARM_GRASP'
    ARM_LIFT = 'ARM_LIFT'
    ARM_STOW = 'ARM_STOW'
    NAVIGATING_PLACE = 'NAVIGATING_PLACE'
    STABILIZING_PLACE = 'STABILIZING_PLACE'
    ARM_RELEASE = 'ARM_RELEASE'
    ARM_FINAL_STOW = 'ARM_FINAL_STOW'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'


class SceneMission(Node):
    def __init__(self) -> None:
        super().__init__('scene_mission')
        for name, value in {
            'goal_configured': False, 'goal_x': 0.0, 'goal_y': 0.0, 'goal_yaw': 0.0,
            'place_configured': False, 'place_x': 0.0, 'place_y': 0.0, 'place_yaw': 0.0,
            'map_frame': 'map', 'base_frame': 'base_footprint', 'tf_timeout_sec': 0.2,
            'stable_wait_ms': 500, 'localization_max_age_sec': 2.0,
            'target_timeout_sec': 3.0, 'target_max_age_sec': 0.5, 'arm_calibrated': False,
            'gripper_calibrated': False, 'arm_profile_json': '', 'arm_stage_sec': 0.8,
            'gripper_servo_id': 10, 'target_frame': 'depth_camera_link',
        }.items():
            self.declare_parameter(name, value)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.vision_client = self.create_client(Trigger, '/raicom/vision/refresh')
        self.zero_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.servo_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.status_pub = self.create_publisher(String, '/raicom/scene/status', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_localization, 10)
        self.create_subscription(Bool, '/raicom/aurora/ready', self._on_ready, 10)
        self.create_subscription(PointStamped, '/raicom/aurora/target_xyz', self._on_target, 10)
        self.create_service(Trigger, '/raicom/scene/start', self._on_start)
        self.create_service(Trigger, '/raicom/scene/cancel', self._on_cancel)
        self.create_timer(0.05, self._tick)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.stage = Stage.IDLE
        self.deadline = 0.0
        self.route: str | None = None
        self.nav_goal_handle = None
        self.last_localization_at: float | None = None
        self.target_ready = False
        self.target_xyz = (0.0, 0.0, 0.0)
        self.target_at: float | None = None
        self.refresh_future = None
        self.refresh_sent = False
        self.profile: dict[str, tuple[tuple[int, float], ...]] = {}
        self._publish('IDLE: scene mission is calibration-gated')

    def _publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _on_localization(self, _msg: PoseWithCovarianceStamped) -> None:
        self.last_localization_at = time.monotonic()

    def _on_ready(self, msg: Bool) -> None:
        self.target_ready = bool(msg.data)
        if not self.target_ready:
            self.target_at = None

    def _on_target(self, msg: PointStamped) -> None:
        expected_frame = str(self.get_parameter('target_frame').value)
        if msg.header.frame_id != expected_frame:
            self.target_ready = False
            self.target_at = None
            self._publish(
                f'WAITING_TARGET: rejected point frame {msg.header.frame_id!r}; '
                f'expected {expected_frame!r}'
            )
            return
        self.target_xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        self.target_at = time.monotonic()

    def _fresh(self, stamp: float | None, max_age: float) -> bool:
        return stamp is not None and 0.0 <= time.monotonic() - stamp <= max_age

    def _localization_available(self) -> bool:
        """Use the timestamped map->base TF, not a stale transient-local pose message."""

        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1_000_000_000.0
        now_ros = self.get_clock().now().nanoseconds / 1_000_000_000.0
        age = now_ros - stamp
        return stamp > 0.0 and 0.0 <= age <= float(
            self.get_parameter('localization_max_age_sec').value
        )

    def _config_error(self) -> str | None:
        config = SceneStartConfig(
            goal_configured=bool(self.get_parameter('goal_configured').value),
            place_configured=bool(self.get_parameter('place_configured').value),
            arm_calibrated=bool(self.get_parameter('arm_calibrated').value),
            gripper_calibrated=bool(self.get_parameter('gripper_calibrated').value),
            arm_profile_json=str(self.get_parameter('arm_profile_json').value),
            stable_wait_ms=int(self.get_parameter('stable_wait_ms').value),
            target_timeout_sec=float(self.get_parameter('target_timeout_sec').value),
        )
        error = validate_scene_start(config)
        if error is None:
            self.profile = parse_arm_profile(config.arm_profile_json)
        return error

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.stage not in (Stage.IDLE, Stage.COMPLETE, Stage.FAILED):
            response.success = False
            response.message = f'mission busy: {self.stage.value}'
            return response
        error = self._config_error()
        if error:
            response.success = False
            response.message = error
            self._publish('REJECTED: ' + error)
            return response
        if not self._localization_available():
            response.success = False
            response.message = 'map-to-base localization TF is missing or stale'
            return response
        if not self.nav_client.server_is_ready():
            response.success = False
            response.message = 'Nav2 navigate_to_pose action is unavailable'
            return response
        self._send_goal('pick')
        response.success = True
        response.message = 'scene mission accepted'
        return response

    def _send_goal(self, route: str) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter('map_frame').value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        if route == 'pick':
            x = float(self.get_parameter('goal_x').value)
            y = float(self.get_parameter('goal_y').value)
            yaw = float(self.get_parameter('goal_yaw').value)
            self.stage = Stage.NAVIGATING_PICK
        else:
            x = float(self.get_parameter('place_x').value)
            y = float(self.get_parameter('place_y').value)
            yaw = float(self.get_parameter('place_yaw').value)
            self.stage = Stage.NAVIGATING_PLACE
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.route = route
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._publish(self.stage.value + ': goal submitted')

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._fail('Nav2 goal transport failed: ' + str(exc))
            return
        if not handle.accepted:
            self._fail('Nav2 rejected the goal')
            return
        self.nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self._fail('Nav2 result transport failed: ' + str(exc))
            return
        self.nav_goal_handle = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Nav2 failed with status={status}')
            return
        self.zero_pub.publish(Twist())
        self.deadline = time.monotonic() + int(self.get_parameter('stable_wait_ms').value) / 1000.0
        self.stage = Stage.STABILIZING_PICK if self.route == 'pick' else Stage.STABILIZING_PLACE
        self._publish(self.stage.value + ': zero velocity hold')

    def _begin_target_refresh(self) -> None:
        # A previous run's ready point must never be reused for this run.
        self.target_ready = False
        self.target_at = None
        self.refresh_future = None
        self.refresh_sent = False
        self.deadline = time.monotonic() + 2.0
        self.stage = Stage.REFRESHING_TARGET
        self._publish('REFRESHING_TARGET: requesting one fresh YOLO frame')

    def _on_refresh_result(self, future) -> None:
        if self.stage is not Stage.REFRESHING_TARGET:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fail('YOLO refresh transport failed: ' + str(exc))
            return
        if not response.success:
            self._fail('YOLO refresh rejected: ' + str(response.message))
            return
        self.stage = Stage.WAITING_TARGET
        self.deadline = time.monotonic() + float(self.get_parameter('target_timeout_sec').value)
        self._publish('WAITING_TARGET: Aurora stable target required')

    def _send_arm_stage(self, stage_name: str) -> None:
        command = ServosPosition()
        command.position_unit = 'pulse'
        command.duration = float(self.get_parameter('arm_stage_sec').value)
        command.position = []
        for servo_id, pulse in self.profile[stage_name]:
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = pulse
            command.position.append(servo)
        self.servo_pub.publish(command)
        self.deadline = time.monotonic() + float(self.get_parameter('arm_stage_sec').value)
        self._publish('ARM_' + stage_name.upper() + ': calibrated profile sent')

    def _tick(self) -> None:
        now = time.monotonic()
        if self.stage is Stage.STABILIZING_PICK and now >= self.deadline:
            self._begin_target_refresh()
        elif self.stage is Stage.REFRESHING_TARGET:
            if not self.refresh_sent and self.vision_client.service_is_ready():
                request = Trigger.Request()
                self.refresh_future = self.vision_client.call_async(request)
                self.refresh_future.add_done_callback(self._on_refresh_result)
                self.refresh_sent = True
                self._publish('REFRESHING_TARGET: YOLO request queued')
            elif not self.refresh_sent and now >= self.deadline:
                self._fail('YOLO refresh service is unavailable')
            elif self.refresh_sent and now >= self.deadline:
                self._fail('YOLO refresh response timed out')
        elif self.stage is Stage.WAITING_TARGET:
            x, y, z = self.target_xyz
            age = float('inf') if self.target_at is None else now - self.target_at
            if target_is_usable(ready=self.target_ready, age_sec=age, max_age_sec=float(self.get_parameter('target_max_age_sec').value), x_m=x, y_m=y, z_m=z):
                self.stage = Stage.ARM_PREGRASP
                self._send_arm_stage('pregrasp')
            elif now >= self.deadline:
                self._fail('Aurora target missing, stale, out of bounds, or unstable')
        elif self.stage is Stage.ARM_PREGRASP and now >= self.deadline:
            self.stage = Stage.ARM_DESCEND
            self._send_arm_stage('descend')
        elif self.stage is Stage.ARM_DESCEND and now >= self.deadline:
            self.stage = Stage.ARM_GRASP
            self._send_arm_stage('grasp')
        elif self.stage is Stage.ARM_GRASP and now >= self.deadline:
            self.stage = Stage.ARM_LIFT
            self._send_arm_stage('lift')
        elif self.stage is Stage.ARM_LIFT and now >= self.deadline:
            self.stage = Stage.ARM_STOW
            self._send_arm_stage('stow')
        elif self.stage is Stage.ARM_STOW and now >= self.deadline:
            self._send_goal('place')
        elif self.stage is Stage.STABILIZING_PLACE and now >= self.deadline:
            self.stage = Stage.ARM_RELEASE
            self._send_arm_stage('release')
        elif self.stage is Stage.ARM_RELEASE and now >= self.deadline:
            self.stage = Stage.ARM_FINAL_STOW
            self._send_arm_stage('final_stow')
        elif self.stage is Stage.ARM_FINAL_STOW and now >= self.deadline:
            self.zero_pub.publish(Twist())
            self.stage = Stage.COMPLETE
            self._publish('COMPLETE: object delivered to configured left red region')

    def _fail(self, reason: str) -> None:
        self.zero_pub.publish(Twist())
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
        self.stage = Stage.FAILED
        self._publish('FAILED: ' + reason + '; base stopped and arm sequence halted')

    def _on_cancel(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.stage in (Stage.IDLE, Stage.COMPLETE, Stage.FAILED):
            self.zero_pub.publish(Twist())
            self._publish(f'{self.stage.value}: cancel requested; stop published')
            response.success = True
            response.message = 'mission idle; stop published'
            return response
        self._fail('operator cancelled')
        response.success = True
        response.message = 'scene mission cancelled; stop published'
        return response


def main(args=None) -> None:
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SceneMission()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.zero_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
