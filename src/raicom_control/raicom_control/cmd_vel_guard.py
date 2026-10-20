"""Independent velocity watchdog between RAICOM inputs and the real base."""

import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import Bool, Empty, String

from .guard_logic import (
    AccelerationLimiter, MotionEnableLease, ZERO_COMMAND, WatchdogState,
    sanitize_command,
)
from .speed_limits import GUARD_SPEED_CONTRACT, validate_guard_limits


class CmdVelGuard(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_guard')
        self.declare_parameter('input_topic', '/raicom/teleop_cmd')
        self.declare_parameter('manual_topic', '/raicom/manual_cmd')
        self.declare_parameter('manual_max_linear', 0.7)
        self.declare_parameter('manual_max_angular', 1.5)
        self.declare_parameter('output_topic', '/controller/cmd_vel')
        self.declare_parameter('timeout_ms', 150)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_linear', 0.50)
        self.declare_parameter('max_angular', 0.50)
        self.declare_parameter('speed_contract_version', GUARD_SPEED_CONTRACT,
                               descriptor=ParameterDescriptor(read_only=True),ignore_override=True)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_angular_accel', 0.7)
        self.declare_parameter('manual_max_angular_accel', 2.0)
        self.declare_parameter('motion_enable_topic', '/raicom/motion_enable')
        self.declare_parameter('stop_topic', '/raicom/nav_stop')

        timeout_ms = int(self.get_parameter('timeout_ms').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        manual_angular_accel = float(self.get_parameter('manual_max_angular_accel').value)
        if not 0.0 < manual_angular_accel <= 5.0:
            raise ValueError('manual_max_angular_accel must be in (0, 5]')
        if not 50 <= timeout_ms <= 1000:
            raise ValueError('timeout_ms must be between 50 and 1000')
        if not 5.0 <= publish_rate <= 100.0:
            raise ValueError('publish_rate_hz must be between 5 and 100')
        validate_guard_limits(self.max_linear,self.max_angular)
        validate_guard_limits(float(self.get_parameter('manual_max_linear').value),
                              float(self.get_parameter('manual_max_angular').value))

        self.state = WatchdogState(timeout_ms / 1000.0)
        self.acceleration = AccelerationLimiter(max_linear_accel, max_angular_accel)
        self.nav_angular_accel = max_angular_accel
        self.manual_angular_accel = manual_angular_accel
        self.motion_enable = MotionEnableLease()
        motion_enable_topic = str(self.get_parameter('motion_enable_topic').value)
        stop_topic = str(self.get_parameter('stop_topic').value)
        self.motion_enable_subscription = self.create_subscription(
            String, motion_enable_topic, self._on_motion_enable, 1)
        self.stop_latched = False
        self.stop_subscription = self.create_subscription(
            Empty, stop_topic, self._on_stop, 10)
        self.nav_active = False
        nav_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.nav_subscription = self.create_subscription(
            Bool, '/raicom/navigation_active', self._on_nav_active, nav_qos)
        self._stale_reported = False
        self._input_at = self._lease_at = None
        self._input_count = self._lease_count = 0
        self._input_values = (0., 0., 0.)
        self._diagnostic_at = 0.
        self.diagnostic_publisher = self.create_publisher(String, '/raicom/guard_status', 1)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        # Velocity is state, not an event stream.  Keeping only the newest sample
        # prevents queued stale commands from extending motion after input stops.
        self.publisher = self.create_publisher(Twist, output_topic, 1)
        # Continuous velocity needs the newest sample, not retransmission of a
        # previous writer's history during manual/navigation handover. Keep
        # lease and emergency-stop events reliable, and retain the 150 ms timer.
        velocity_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Twist, input_topic, self._on_command, velocity_qos)
        manual_topic = str(self.get_parameter('manual_topic').value)
        self.manual_subscription = self.create_subscription(
            Twist, manual_topic, self._on_manual_command, velocity_qos)
        # A ROS clock adjustment must never freeze zero-command publication.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_guarded,
                                      clock=self.steady_clock)
        self.add_on_set_parameters_callback(self._validate_speed_parameters)
        self.get_logger().info(
            f'guard active: {input_topic} -> {output_topic}, timeout={timeout_ms} ms, '
            f'limits={self.max_linear:.2f} m/s and {self.max_angular:.2f} rad/s, '
            f'accel={max_linear_accel:.2f} m/s2 and {max_angular_accel:.2f} rad/s2; '
            f'motion enable topic={motion_enable_topic}; stop topic={stop_topic}; '
            'navigation lateral disabled; manual lease permits lateral'
        )

    @property
    def max_linear(self):
        return float(self.get_parameter('max_linear').value)

    @property
    def max_angular(self):
        return float(self.get_parameter('max_angular').value)

    def _validate_speed_parameters(self, parameters):
        # Validation only. Humble commits parameters AFTER callbacks succeed;
        # command processing reads committed values instead of a speculative cache.
        changes={p.name:p.value for p in parameters}
        fixed={'timeout_ms','publish_rate_hz','max_linear_accel','max_angular_accel',
               'manual_max_angular_accel',
               'input_topic','manual_topic','manual_max_linear','manual_max_angular',
               'output_topic','motion_enable_topic','stop_topic','speed_contract_version'}
        if any(name in changes and changes[name]!=self.get_parameter(name).value for name in fixed):
            return SetParametersResult(successful=False,reason='watchdog configuration requires restart')
        if not changes.keys() & {'max_linear','max_angular'}:
            return SetParametersResult(successful=True)
        try:validate_guard_limits(changes.get('max_linear',self.max_linear),changes.get('max_angular',self.max_angular))
        except ValueError as exc:return SetParametersResult(successful=False,reason=str(exc))
        now=time.monotonic()
        if self._motion_enabled() or not self.state.command_at(now).is_zero or not self.acceleration.previous.is_zero:
            return SetParametersResult(successful=False,reason='speed update requires disabled motion lease and zero output')
        return SetParametersResult(successful=True)

    def _on_motion_enable(self, msg):
        previous_mission = self.motion_enable.mission_id
        self._lease_count = getattr(self, '_lease_count', 0) + 1
        self._lease_at = time.monotonic()
        try:
            message = json.loads(msg.data)
        except (ValueError, TypeError):
            message = None
        now = time.monotonic()
        if not self.motion_enable.receive(message, now) or not self.motion_enable.valid_at(now):
            self.acceleration.reset(now)
            self.state.update(ZERO_COMMAND, now)
            return
        # A fresh explicit enable is the only software reset for the current
        # stop latch.  No velocity message can clear an emergency stop.
        self.stop_latched = False
        if self.motion_enable.mission_id != previous_mission:
            self.acceleration.reset(now)
            self.state.update(ZERO_COMMAND, now)

    def _on_nav_active(self, message):
        was_active = self.nav_active
        self.nav_active = bool(message.data)
        # The bridge republishes its active state every 50 ms.  Reset only
        # during the handoff into navigation; resetting for every heartbeat
        # keeps the acceleration limiter near zero indefinitely.
        if self.nav_active and not was_active:
            now = time.monotonic()
            self.acceleration.reset(now)
            self.state.update(ZERO_COMMAND, now)

    def _manual_lease(self):
        return (self.motion_enable.valid_at(time.monotonic())
                and str(self.motion_enable.mission_id).startswith('competition_manual:'))

    def _manual_active(self):
        return self._manual_lease() and not self.nav_active

    def _on_stop(self, _msg):
        self.stop_latched = True
        now = time.monotonic()
        self.acceleration.reset(now)
        self.state.update(ZERO_COMMAND, now)

    def _motion_enabled(self):
        return self.motion_enable.valid_at(time.monotonic())

    def _on_command(self, msg: Twist) -> None:
        if self._manual_lease():
            return
        self._process_command(msg, allow_lateral=False)

    def _on_manual_command(self, msg: Twist) -> None:
        if self._manual_active():
            self._process_command(msg, allow_lateral=True)

    def _process_command(self, msg: Twist, allow_lateral: bool) -> None:
        now = time.monotonic()
        self._input_count = getattr(self, '_input_count', 0) + 1
        self._input_at = now
        self._input_values = (msg.linear.x, msg.linear.y, msg.angular.z)
        # Operating restriction requested after repeatable lateral execution
        # faults. Reject the whole command, including mixed-axis requests;
        # silently projecting it forward would change the requested path.
        if msg.linear.y != 0.0 and not allow_lateral:
            self.acceleration.reset(now)
            self.state.update(ZERO_COMMAND, now)
            self._stale_reported = False
            return
        if self.state.is_stale(now):
            self.acceleration.reset(now)
        command = sanitize_command(
            (msg.linear.x, msg.linear.y, msg.angular.z),
            float(self.get_parameter('manual_max_linear').value) if allow_lateral else self.max_linear,
            float(self.get_parameter('manual_max_angular').value) if allow_lateral else self.max_angular
        )
        if self.stop_latched or not self._motion_enabled():
            self.acceleration.reset(now)
            command = ZERO_COMMAND
        elif allow_lateral and command.is_zero:
            self.acceleration.reset(now)
        else:
            self.acceleration.max_angular_accel = (
                self.manual_angular_accel if allow_lateral else self.nav_angular_accel)
            command = self.acceleration.apply(command, now)
        self.state.update(command, now)
        self._stale_reported = False

    @staticmethod
    def _to_msg(command) -> Twist:
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.linear.y = command.linear_y
        msg.angular.z = command.angular_z
        return msg

    def _publish_guarded(self) -> None:
        now = time.monotonic()
        command = self.state.command_at(now)
        if (self.stop_latched or not self._motion_enabled()
                or (self.nav_active and self._manual_lease())
                or self.state.is_stale(now)):
            command = ZERO_COMMAND
            self.acceleration.reset(now)
            self.state.update(ZERO_COMMAND, now)
        self.publisher.publish(self._to_msg(command))
        if getattr(self, 'diagnostic_publisher', None) is not None and now-self._diagnostic_at >= .5:
            self._diagnostic_at = now
            self.diagnostic_publisher.publish(String(data=json.dumps(self.diagnostic_snapshot(now, command))))
        if self.state.is_stale(now) and not self._stale_reported:
            self.get_logger().warn('input stale or absent; continuously publishing zero velocity')
            self._stale_reported = True

    def diagnostic_snapshot(self, now, command):
        """Report the actual callback state; this never grants motion."""
        input_at = getattr(self, '_input_at', None)
        lease_at = getattr(self, '_lease_at', None)
        incoming = getattr(self, '_input_values', (0., 0., 0.))
        if self.stop_latched: reason = 'stop_latched'
        elif not self.motion_enable.valid_at(now):
            reason = 'lease_absent' if lease_at is None else 'lease_disabled_or_expired'
        elif input_at is None: reason = 'input_absent'
        elif now-input_at > self.state.timeout_sec: reason = 'input_expired'
        elif incoming[1] != 0.: reason = 'lateral_rejected'
        elif command.is_zero: reason = 'zero_input_or_ramp'
        else: reason = 'passing'
        return {'reason': reason, 'monotonic': now, 'input_count': getattr(self, '_input_count', 0),
                'lease_count': getattr(self, '_lease_count', 0),
                'input_age': None if input_at is None else now-input_at,
                'lease_age': None if lease_at is None else now-lease_at,
                'lease_enabled': self.motion_enable.enabled, 'lease_expires_at': self.motion_enable.expires_at,
                'mission_id': self.motion_enable.mission_id, 'stop_latched': self.stop_latched,
                'input': incoming, 'output': [command.linear_x, command.linear_y, command.angular_z]}

    def publish_zero(self) -> None:
        self.publisher.publish(Twist())


def main(args=None) -> None:
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CmdVelGuard()
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
