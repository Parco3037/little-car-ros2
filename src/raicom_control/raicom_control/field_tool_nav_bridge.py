"""Route Nav2 velocity commands through the existing independent watchdog for the field tool."""

import json
import signal
import time
from pathlib import Path

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

from .field_tool_nav_gate import NavigationCommandGate


class NavCmdBridge(Node):
    """Forward Nav2 output to the watchdog input without touching the base."""

    def __init__(self) -> None:
        super().__init__('nav_cmd_bridge')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/raicom/teleop_cmd')
        self.declare_parameter('stop_topic', '/raicom/nav_stop')
        self.declare_parameter('active_topic', '/raicom/navigation_active')
        self.declare_parameter('manual_suppressed_topic', '/raicom/manual_suppressed')
        self.declare_parameter('goal_context_topic', '/raicom/nav_goal_context')
        self.declare_parameter('boot_id', '')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        stop_topic = str(self.get_parameter('stop_topic').value)
        active_topic = str(self.get_parameter('active_topic').value)
        suppressed_topic = str(self.get_parameter('manual_suppressed_topic').value)
        goal_context_topic = str(self.get_parameter('goal_context_topic').value)
        boot_id = str(self.get_parameter('boot_id').value)
        if not boot_id:
            try:
                boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            except OSError:
                boot_id = ''
        self.gate = NavigationCommandGate(expected_boot_id=boot_id or None)
        self.publisher = self.create_publisher(Twist, output_topic, 1)
        self.subscription = self.create_subscription(
            Twist, input_topic, self._on_command, 1
        )
        self.stop_subscription = self.create_subscription(
            Empty, stop_topic, self._on_stop, 10)
        self.suppressed_subscription = self.create_subscription(
            Bool, suppressed_topic, self._on_manual_suppressed, 10)
        self.goal_context_subscription = self.create_subscription(
            String, goal_context_topic, self._on_goal_context, 1)
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.active_publisher = self.create_publisher(Bool, active_topic, latched_qos)
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.follow_path_cancel_client = self.create_client(
            CancelGoal, '/follow_path/_action/cancel_goal')
        self.timer = self.create_timer(0.05, self._tick)
        self.was_active = False
        self._publish_active(False)
        self.get_logger().info(
            f'Nav2 adapter active: {input_topic} -> {output_topic} -> watchdog; '
            f'goal context={goal_context_topic}; Space stops only the current goal'
        )

    def _publish_active(self, active: bool) -> None:
        self.active_publisher.publish(Bool(data=active))

    def _on_manual_suppressed(self, message: Bool) -> None:
        self.gate.observe_manual_suppressed(message.data, time.monotonic())

    def _on_goal_context(self, message: String) -> None:
        try:
            context = json.loads(message.data)
        except (TypeError, ValueError):
            context = None
        now = time.monotonic()
        if not self.gate.receive_goal_context(context, now):
            self.publisher.publish(Twist())
            return
        self._publish_active(self.gate.has_active_goal(now))

    def _on_command(self, message: Twist) -> None:
        now = time.monotonic()
        if self.gate.has_active_goal(now):
            self.publisher.publish(message if self.gate.permits_motion(now) else Twist())

    def _on_stop(self, _message: Empty) -> None:
        first = self.gate.request_stop()
        self.publisher.publish(Twist())
        self._publish_active(False)
        if first:
            self.get_logger().error(
                'OPERATOR SPACE STOPPED CURRENT GOAL; next goal needs a fresh context')
            available = False
            for client in (self.cancel_client, self.follow_path_cancel_client):
                if client.service_is_ready():
                    client.call_async(CancelGoal.Request())
                    available = True
            if not available:
                self.get_logger().error('Navigation cancel services unavailable; zero latch remains active')

    def _tick(self) -> None:
        now = time.monotonic()
        active = self.gate.has_active_goal(now)
        self._publish_active(active)
        if (active and not self.gate.permits_motion(now)) or (self.was_active and not active):
            self.publisher.publish(Twist())
        self.was_active = active

    def publish_zero(self) -> None:
        self.publisher.publish(Twist())

    def release_manual_input(self) -> None:
        self._publish_active(False)


def main(args=None) -> None:
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavCmdBridge()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.publish_zero()
                time.sleep(0.02)
            node.release_manual_input()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
