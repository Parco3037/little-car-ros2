"""Route Nav2 velocity commands through the independent RAICOM watchdog."""

import signal
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Empty

from .nav_control_logic import NavigationCommandGate


class NavCmdBridge(Node):
    """Forward Nav2 output to the watchdog input without touching the base."""

    def __init__(self) -> None:
        super().__init__('nav_cmd_bridge')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/raicom/teleop_cmd')
        self.declare_parameter('stop_topic', '/raicom/nav_stop')
        self.declare_parameter('active_topic', '/raicom/navigation_active')
        self.declare_parameter('manual_suppressed_topic', '/raicom/manual_suppressed')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        stop_topic = str(self.get_parameter('stop_topic').value)
        active_topic = str(self.get_parameter('active_topic').value)
        suppressed_topic = str(self.get_parameter('manual_suppressed_topic').value)
        self.gate = NavigationCommandGate()
        self.publisher = self.create_publisher(Twist, output_topic, 1)
        self.subscription = self.create_subscription(
            Twist, input_topic, self._on_command, 1
        )
        self.stop_subscription = self.create_subscription(
            Empty, stop_topic, self._on_stop, 10)
        self.suppressed_subscription = self.create_subscription(
            Bool, suppressed_topic, self._on_manual_suppressed, 10)
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.active_publisher = self.create_publisher(Bool, active_topic, latched_qos)
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.timer = self.create_timer(0.05, self._tick)
        self._publish_active(True)
        self.get_logger().info(
            f'Nav2 safety route waiting for fresh manual suppression: '
            f'{input_topic} -> {output_topic} -> watchdog; Space latches stop'
        )

    def _publish_active(self, active: bool) -> None:
        self.active_publisher.publish(Bool(data=active))

    def _on_manual_suppressed(self, message: Bool) -> None:
        self.gate.observe_manual_suppressed(message.data, time.monotonic())

    def _on_command(self, message: Twist) -> None:
        self.publisher.publish(message if self.gate.permits_motion(time.monotonic()) else Twist())

    def _on_stop(self, _message: Empty) -> None:
        first = self.gate.request_stop()
        self.publisher.publish(Twist())
        if first:
            self.get_logger().error('OPERATOR SPACE STOP LATCHED; bridge restart required')
            if self.cancel_client.service_is_ready():
                self.cancel_client.call_async(CancelGoal.Request())
            else:
                self.get_logger().error('NavigateToPose cancel service unavailable; zero latch remains active')

    def _tick(self) -> None:
        self._publish_active(True)
        if not self.gate.permits_motion(time.monotonic()):
            self.publisher.publish(Twist())

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
