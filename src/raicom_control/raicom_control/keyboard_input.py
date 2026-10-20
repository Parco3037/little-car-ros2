"""Terminal keyboard input that only publishes to the guarded RAICOM topic."""

import select
import sys
import termios
import time
import tty
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger


KEY_COMMANDS = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
}
MISSION_START_KEY = 'm'


class KeyboardInput(Node):
    def __init__(self, *, gui_only: bool | None = None) -> None:
        super().__init__('keyboard_input')
        self.declare_parameter('output_topic', '/raicom/teleop_cmd')
        self.declare_parameter('key_topic', '/raicom/teleop_key')
        self.declare_parameter('nav_stop_topic', '/raicom/nav_stop')
        self.declare_parameter('navigation_active_topic', '/raicom/navigation_active')
        self.declare_parameter('manual_suppressed_topic', '/raicom/manual_suppressed')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('command_hold_ms', 200)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('gui_only', False)
        if gui_only is not None:
            self.set_parameters([
                Parameter('gui_only', value=bool(gui_only)),
            ])

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.command_hold_sec = int(self.get_parameter('command_hold_ms').value) / 1000.0
        self.gui_only = bool(self.get_parameter('gui_only').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if not 0.0 < self.linear_speed <= 0.50:
            raise ValueError('linear_speed exceeds the 0.50 m/s manual-mapping limit')
        if not 0.0 < self.angular_speed <= 0.50:
            raise ValueError('angular_speed exceeds the 0.50 rad/s operating limit')
        if not 0.05 <= self.command_hold_sec <= 0.25:
            raise ValueError('command_hold_ms must be between 50 and 250')

        # ros2 launch does not always pass the parent PTY to a child process.
        # In the vendor launch path, /dev/tty is still the controlling terminal
        # exposed by `docker exec -it`; use it when stdin is a launch pipe.
        self._tty_fd = None
        self._terminal_settings = None
        if self.gui_only:
            self._input_fd = -1
            self._terminal_input_closed = True
        else:
            self._input_fd = sys.stdin.fileno()
            self._terminal_input_closed = False
            if not sys.stdin.isatty():
                try:
                    self._tty_fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
                except OSError as exc:
                    raise RuntimeError(
                        'keyboard_input needs an interactive terminal or /dev/tty'
                    ) from exc
                if not os.isatty(self._tty_fd):
                    os.close(self._tty_fd)
                    self._tty_fd = None
                    raise RuntimeError('keyboard_input could not open a controlling terminal')
                self._input_fd = self._tty_fd

        output_topic = str(self.get_parameter('output_topic').value)
        key_topic = str(self.get_parameter('key_topic').value)
        nav_stop_topic = str(self.get_parameter('nav_stop_topic').value)
        active_topic = str(self.get_parameter('navigation_active_topic').value)
        suppressed_topic = str(self.get_parameter('manual_suppressed_topic').value)
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.nav_stop_publisher = self.create_publisher(Empty, nav_stop_topic, 10)
        self.manual_suppressed_publisher = self.create_publisher(Bool, suppressed_topic, 10)
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.navigation_active_subscription = self.create_subscription(
            Bool, active_topic, self._on_navigation_active, latched_qos)
        # The GUI panel publishes key events here.  Keeping the final Twist
        # publisher in this node preserves one and only one manual speed
        # publisher and the same watchdog path as terminal keyboard input.
        self.key_subscription = self.create_subscription(
            String, key_topic, self._on_key_message, 10
        )
        self.mission_client = self.create_client(Trigger, '/raicom/scene/start')
        self._mission_future = None
        self.current = Twist()
        self.last_key_at: float | None = None
        self.exit_requested = False
        self.navigation_active = False
        self._terminal_settings = None
        if not self.gui_only:
            self._terminal_settings = termios.tcgetattr(self._input_fd)
            tty.setraw(self._input_fd)
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(
            'W/S forward/back, Q/E rotate, A/D disabled, Space stop, M start scene mission, '
            'Ctrl+C stop and exit'
        )

    def _read_keys(self) -> list[str]:
        if self._terminal_input_closed:
            return []
        keys = []
        try:
            while select.select([self._input_fd], [], [], 0.0)[0]:
                data = os.read(self._input_fd, 1)
                if data:
                    keys.append(data.decode(errors='ignore'))
                    continue
                # A desktop terminal can disappear while the ROS launch tree
                # remains alive.  Do not spin forever on an EOF-ready tty:
                # GUI key events must still reach this node.
                self._terminal_input_closed = True
                self.get_logger().warn(
                    'terminal input closed; continuing with GUI key events'
                )
                break
        except OSError as exc:
            self._terminal_input_closed = True
            self.get_logger().warn(
                f'terminal input unavailable ({exc}); continuing with GUI key events'
            )
        return keys

    def _on_key_message(self, message: String) -> None:
        """Accept a single user-generated key from the optional GUI panel."""

        if message.data == 'release':
            # Task handoff releases teleop only. Space is a global emergency
            # stop and would otherwise immediately cancel the new arm task.
            self.current = Twist()
            self.last_key_at = None
            self.publisher.publish(self.current)
            return
        if message.data:
            self._set_command(message.data[0])

    def _set_command(self, key: str) -> None:
        if key == ' ':
            self.current = Twist()
            self.last_key_at = None
            self.nav_stop_publisher.publish(Empty())
            return
        if self.navigation_active:
            return
        if key.lower() == MISSION_START_KEY:
            self._request_scene_start()
            return
        axes = KEY_COMMANDS.get(key.lower())
        if axes is None:
            return
        if axes[1] != 0.0:
            # A disabled direction must also release any previously held key.
            self.current = Twist()
            self.last_key_at = None
            self.publisher.publish(self.current)
            return
        msg = Twist()
        msg.linear.x = axes[0] * self.linear_speed
        msg.linear.y = axes[1] * self.linear_speed
        msg.angular.z = axes[2] * self.angular_speed
        self.current = msg
        self.last_key_at = time.monotonic()

    def _request_scene_start(self) -> None:
        """Request the gated mission; this key never publishes motion or servo data."""

        # Release manual teleop before handing control to Nav2.  The guard also
        # enforces its timeout, but clearing here removes the hand-off overlap.
        self.current = Twist()
        self.last_key_at = None
        if self._mission_future is not None and not self._mission_future.done():
            self.get_logger().warn('scene mission start already pending')
            return
        if not self.mission_client.service_is_ready():
            self.get_logger().warn('scene mission service is unavailable; no action taken')
            return
        self._mission_future = self.mission_client.call_async(Trigger.Request())
        self._mission_future.add_done_callback(self._on_scene_start_response)

    def _on_scene_start_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - ROS transport failure path
            self.get_logger().error(f'scene mission start failed: {exc}')
            return
        if response.success:
            self.get_logger().info('scene mission accepted: ' + str(response.message))
        else:
            self.get_logger().warn('scene mission rejected: ' + str(response.message))

    def _tick(self) -> None:
        for key in self._read_keys():
            if key == '\x03':
                self.current = Twist()
                self.publisher.publish(self.current)
                self.exit_requested = True
                return
            self._set_command(key)

        self.manual_suppressed_publisher.publish(Bool(data=self.navigation_active))
        if self.navigation_active:
            return

        if self.last_key_at is None or time.monotonic() - self.last_key_at > self.command_hold_sec:
            self.current = Twist()
        self.publisher.publish(self.current)

    def _on_navigation_active(self, message: Bool) -> None:
        active = bool(message.data)
        if active and not self.navigation_active:
            self.current = Twist()
            self.last_key_at = None
            self.publisher.publish(self.current)
            self.get_logger().warn('navigation active: manual motion keys suppressed; Space remains enabled')
        self.navigation_active = active

    def restore_terminal(self) -> None:
        if self.gui_only:
            return
        try:
            termios.tcsetattr(self._input_fd, termios.TCSADRAIN, self._terminal_settings)
        except OSError:
            pass
        if self._tty_fd is not None:
            os.close(self._tty_fd)
            self._tty_fd = None

    def publish_zero(self) -> None:
        if rclpy.ok():
            self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardInput()
    try:
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
