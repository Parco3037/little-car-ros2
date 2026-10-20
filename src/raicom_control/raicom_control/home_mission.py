"""Safety-gated one-shot home navigation and gripper-only arrival action."""

from enum import Enum
import math
import signal
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from servo_controller_msgs.msg import ServoPosition, ServosPosition
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .mission_logic import (
    requires_joint_state,
    should_run_arrival_action,
    validate_mission_configuration,
)


class MissionStage(Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    STABILIZING = 'STABILIZING'
    GRIPPER_OPEN = 'GRIPPER_OPEN'
    GRIPPER_CLOSE = 'GRIPPER_CLOSE'
    FAILED = 'FAILED'


class HomeMission(Node):
    def __init__(self) -> None:
        super().__init__('home_mission')
        self.declare_parameter('goal_configured', False)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('stable_wait_ms', 500)
        self.declare_parameter('localization_max_age_sec', 2.0)
        self.declare_parameter('arrival_action_enabled', False)
        self.declare_parameter('gripper_calibrated', False)
        self.declare_parameter('gripper_servo_id', 10)
        self.declare_parameter('gripper_open_pulse', 0.0)
        self.declare_parameter('gripper_close_pulse', 0.0)
        self.declare_parameter('gripper_max_delta_pulse', 80.0)
        self.declare_parameter('gripper_duration_sec', 0.5)
        self.declare_parameter('joint_state_max_age_sec', 2.0)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.zero_publisher = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.servo_publisher = self.create_publisher(ServosPosition, '/servo_controller', 10)
        self.status_publisher = self.create_publisher(String, '/raicom/home/status', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_localization, 10
        )
        self.create_subscription(
            JointState, '/controller_manager/joint_states', self._on_joint_state, 10
        )
        self.create_service(Trigger, '/raicom/home/start', self._on_start)
        self.create_service(Trigger, '/raicom/home/cancel', self._on_cancel)
        self.create_timer(0.05, self._tick)

        self.stage = MissionStage.IDLE
        self.stage_deadline = 0.0
        self.last_localization_at: float | None = None
        self.last_joint_state_at: float | None = None
        self.nav_goal_handle = None
        self._publish_status('IDLE: waiting for a configured home mission')

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)
        self.get_logger().info(text)

    def _on_localization(self, _msg: PoseWithCovarianceStamped) -> None:
        self.last_localization_at = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        if all(name in msg.name for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')):
            self.last_joint_state_at = time.monotonic()

    def _fresh(self, timestamp: float | None, max_age: float) -> bool:
        return timestamp is not None and 0.0 <= time.monotonic() - timestamp <= max_age

    def _configuration_error(self) -> str | None:
        return validate_mission_configuration(
            goal_configured=bool(self.get_parameter('goal_configured').value),
            arrival_action_enabled=bool(
                self.get_parameter('arrival_action_enabled').value
            ),
            gripper_calibrated=bool(self.get_parameter('gripper_calibrated').value),
            gripper_open_pulse=float(self.get_parameter('gripper_open_pulse').value),
            gripper_close_pulse=float(self.get_parameter('gripper_close_pulse').value),
            gripper_max_delta_pulse=float(
                self.get_parameter('gripper_max_delta_pulse').value
            ),
        )

    def _on_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.stage is not MissionStage.IDLE:
            response.success = False
            response.message = f'mission busy: {self.stage.value}'
            return response
        error = self._configuration_error()
        if error:
            response.success = False
            response.message = error
            return response
        if not self._fresh(
            self.last_localization_at, float(self.get_parameter('localization_max_age_sec').value)
        ):
            response.success = False
            response.message = 'localization is missing or stale'
            return response
        if requires_joint_state(
            bool(self.get_parameter('arrival_action_enabled').value)
        ):
            if not self._fresh(
                self.last_joint_state_at,
                float(self.get_parameter('joint_state_max_age_sec').value),
            ):
                response.success = False
                response.message = 'arm/gripper joint state is missing or stale'
                return response
        if not self.nav_client.server_is_ready():
            response.success = False
            response.message = 'Nav2 navigate_to_pose action is unavailable'
            return response

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter('map_frame').value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.pose.pose.position.y = float(self.get_parameter('goal_y').value)
        yaw = float(self.get_parameter('goal_yaw').value)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.stage = MissionStage.NAVIGATING
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._publish_status('NAVIGATING: goal submitted')
        response.success = True
        response.message = 'home mission accepted'
        return response

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - ROS transport failure path
            self._fail(f'Nav2 goal transport failed: {exc}')
            return
        if not goal_handle.accepted:
            self._fail('Nav2 rejected the goal')
            return
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:  # pragma: no cover - ROS transport failure path
            self._fail(f'Nav2 result transport failed: {exc}')
            return
        self.nav_goal_handle = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Nav2 did not succeed, status={status}')
            return
        self.zero_publisher.publish(Twist())
        wait_sec = int(self.get_parameter('stable_wait_ms').value) / 1000.0
        self.stage = MissionStage.STABILIZING
        self.stage_deadline = time.monotonic() + wait_sec
        self._publish_status(f'STABLIZING: zero velocity for {wait_sec:.3f} s')

    def _publish_gripper(self, pulse: float) -> None:
        servo = ServoPosition()
        servo.id = int(self.get_parameter('gripper_servo_id').value)
        servo.position = pulse
        command = ServosPosition()
        command.duration = float(self.get_parameter('gripper_duration_sec').value)
        command.position = [servo]
        self.servo_publisher.publish(command)

    def _tick(self) -> None:
        now = time.monotonic()
        if self.stage is MissionStage.STABILIZING and now >= self.stage_deadline:
            if not should_run_arrival_action(
                bool(self.get_parameter('arrival_action_enabled').value)
            ):
                self.zero_publisher.publish(Twist())
                self.stage = MissionStage.IDLE
                self._publish_status(
                    'IDLE: navigation completed; arrival action remained disabled'
                )
                return
            pulse = float(self.get_parameter('gripper_open_pulse').value)
            self._publish_gripper(pulse)
            self.stage = MissionStage.GRIPPER_OPEN
            self.stage_deadline = now + float(self.get_parameter('gripper_duration_sec').value) + 0.2
            self._publish_status('GRIPPER_OPEN: ID10 only; joints 1-5 unchanged')
        elif self.stage is MissionStage.GRIPPER_OPEN and now >= self.stage_deadline:
            pulse = float(self.get_parameter('gripper_close_pulse').value)
            self._publish_gripper(pulse)
            self.stage = MissionStage.GRIPPER_CLOSE
            self.stage_deadline = now + float(self.get_parameter('gripper_duration_sec').value) + 0.2
            self._publish_status('GRIPPER_CLOSE: ID10 only; joints 1-5 unchanged')
        elif self.stage is MissionStage.GRIPPER_CLOSE and now >= self.stage_deadline:
            self.zero_publisher.publish(Twist())
            self.stage = MissionStage.IDLE
            self._publish_status('IDLE: mission completed once')

    def _fail(self, reason: str) -> None:
        self.zero_publisher.publish(Twist())
        self.stage = MissionStage.IDLE
        self._publish_status(f'FAILED: {reason}; robot commanded to stop')

    def _on_cancel(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.zero_publisher.publish(Twist())
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        was_active = self.stage is not MissionStage.IDLE
        self.stage = MissionStage.IDLE
        self.nav_goal_handle = None
        self._publish_status('IDLE: mission cancelled; robot commanded to stop')
        response.success = True
        response.message = 'active mission cancelled' if was_active else 'already idle; stop published'
        return response


def main(args=None) -> None:
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = HomeMission()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.zero_publisher.publish(Twist())
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
