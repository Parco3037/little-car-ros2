"""Xbox input layer for the vendor controller and RAICOM guard."""

import json
import math
import os
import signal
import subprocess
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from servo_controller_msgs.msg import ServoPosition, ServosPosition
from std_msgs.msg import Bool, String


POSES = {
    0: {2: 100, 3: 150, 4: 850, 5: 875},  # A
    1: {1: 450, 2: 500, 3: 100, 4: 550, 5: 875},  # B: man2
    2: {1: 450, 2: 500, 3: 250, 4: 400, 5: 875},  # X: man1
    3: {1: 450, 2: 425, 3: 360, 4: 390, 5: 875},  # Y: man3
    4: {1: 450, 2: 380, 3: 455, 4: 465, 5: 875},  # LT: man4
}
A_SECOND_POSE = {2: 190, 3: 500, 4: 440, 5: 855}
INIT_POSE = {1: 450, 2: 820, 3: 175, 4: 90, 5: 875}


def deadzone(value, threshold):
    if not math.isfinite(value) or abs(value) < threshold:
        return 0.0
    return max(-1.0, min(1.0, value))


def stick_velocity(x, y, threshold, max_x, max_y):
    if not math.isfinite(x) or not math.isfinite(y):
        return 0.0, 0.0
    x, y = deadzone(x, threshold), deadzone(y, threshold)
    magnitude = math.hypot(x, y)
    if magnitude <= threshold:
        return 0.0, 0.0
    fraction = (min(magnitude, 1.0) - threshold) / (1.0 - threshold)
    return (x / magnitude * (0.1 + (max_x - 0.1) * fraction),
            y / magnitude * (0.1 + (max_y - 0.1) * fraction))


class XboxTeleop(Node):
    def __init__(self):
        super().__init__('competition_xbox_teleop')
        for name, default in (
            ('max_linear_x', 0.5), ('max_linear_y', 0.5),
            ('max_angular_z', 1.5), ('deadzone', 0.05),
            ('joy_timeout_sec', 0.25),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter('start_sound', '')
        self.declare_parameter('audio_output_device', 'plughw:CARD=Device,DEV=0')
        self.linear_x = float(self.get_parameter('max_linear_x').value)
        self.linear_y = float(self.get_parameter('max_linear_y').value)
        self.angular_z = float(self.get_parameter('max_angular_z').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout_sec').value)
        self.start_sound = str(self.get_parameter('start_sound').value)
        self.audio_output_device = str(self.get_parameter('audio_output_device').value)
        if not (0.1 <= self.linear_x <= 0.7 and 0.1 <= self.linear_y <= 0.7
                and 0 < self.angular_z <= 1.5 and 0 <= self.deadzone < 0.5
                and 0.1 <= self.joy_timeout <= 1.0):
            raise ValueError('manual speed, deadzone or Joy timeout outside safe limits')

        self.velocity_pub = self.create_publisher(Twist, '/raicom/manual_cmd', 1)
        self.lease_pub = self.create_publisher(String, '/raicom/motion_enable', 1)
        self.servo_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        nav_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.voice_enable_pub = self.create_publisher(
            Bool, '/competition_manual/voice_enable', nav_qos)
        self.create_subscription(Bool, '/raicom/navigation_active',
                                 self.on_navigation, nav_qos)
        self.create_subscription(Bool, '/competition_manual/auto_requested',
                                 self.on_auto_requested, nav_qos)
        self.create_subscription(Bool, '/competition_manual/joy_connected',
                                 self.on_joy_connected, nav_qos)
        self.navigation_active = False
        self.manual_locked = False
        self.joy_connected = False
        self.rearm_required = True
        self.last_joy_at = None
        self.target = Twist()
        self.last_buttons = None
        self.last_start = None
        self.current_pose = {1: 460, 2: 500, 3: 150, 4: 500, 5: 875, 10: 300}
        self.claw_closed = False
        self.active_pose = None  # Last successfully published preset, not readback.
        self.a_second_pose_next = False
        self.mission_id = 'competition_manual:' + str(uuid.uuid4())
        try:
            with open('/proc/sys/kernel/random/boot_id', encoding='ascii') as source:
                self.boot_id = source.read().strip()
        except OSError:
            self.boot_id = str(uuid.uuid4())
        self.last_warning_at = 0.0
        self.create_timer(0.05, self.publish_control)
        self.get_logger().info('Xbox input ready; waiting for fresh /joy')

    def on_navigation(self, message):
        self.navigation_active = bool(message.data)
        if self.navigation_active and not self.manual_locked:
            self.target = Twist()
            self.publish_zero()

    def on_joy_connected(self, message):
        self.joy_connected = bool(message.data)
        if not self.joy_connected:
            self.rearm_required = True
            self.target = Twist()
            self.last_buttons = None
            self.last_start = None
            self.publish_zero()
            self.publish_lease(False)

    def on_auto_requested(self, message):
        if not message.data or self.manual_locked:
            return
        self.manual_locked = True
        self.target = Twist()
        for _ in range(3):
            self.publish_zero()
            self.publish_lease(False)
            time.sleep(0.02)
        self.destroy_publisher(self.velocity_pub)
        self.destroy_publisher(self.lease_pub)
        self.destroy_publisher(self.servo_pub)
        self.velocity_pub = self.lease_pub = self.servo_pub = None
        self.get_logger().warn('auto mode requested: gamepad chassis and arm locked until launch restart')

    def owners_ok(self):
        base_owners = self.get_publishers_info_by_topic('/controller/cmd_vel')
        if len(base_owners) != 1 or base_owners[0].node_name != 'cmd_vel_guard':
            return False
        node_names = set(self.get_node_names())
        return not node_names.intersection({'nav_cmd_bridge', 'raicom_field_tool'})

    def arm_owners_ok(self):
        owners = self.get_publishers_info_by_topic('/servo_controller')
        return (len(owners) == 1
                and owners[0].node_name == self.get_name())

    def joy_owners_ok(self):
        owners = self.get_publishers_info_by_topic('/joy')
        return (len(owners) == 1 and owners[0].node_name == 'xbox_joy'
                and 'competition_joy_reconnect' in self.get_node_names())

    def publish_lease(self, enabled):
        if self.lease_pub is None:
            return
        payload = {
            'boot_id': self.boot_id,
            'mission_id': self.mission_id,
            'enabled': bool(enabled),
            'expires_at': time.monotonic() + (0.20 if enabled else 0.0),
        }
        self.lease_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def publish_zero(self):
        if self.velocity_pub is not None:
            self.velocity_pub.publish(Twist())

    def publish_control(self):
        if self.manual_locked:
            return
        now = time.monotonic()
        fresh = self.last_joy_at is not None and now - self.last_joy_at <= self.joy_timeout
        if not fresh:
            self.rearm_required = True
            self.target = Twist()
        enabled = (fresh and self.joy_connected and not self.rearm_required
                   and not self.navigation_active and self.joy_owners_ok() and self.owners_ok())
        self.publish_lease(enabled)
        self.velocity_pub.publish(self.target if enabled else Twist())
        if not enabled and now - self.last_warning_at > 5.0:
            self.last_warning_at = now
            self.get_logger().warn('motion inhibited: Joy stale, navigation active, or controller owners conflict')

    def on_joy(self, message):
        if self.manual_locked or not self.joy_connected or not self.joy_owners_ok():
            return
        self.last_joy_at = time.monotonic()
        axes = list(message.axes)
        buttons = list(message.buttons)
        if len(axes) < 8 or len(buttons) < 12:
            self.rearm_required = True
            self.target = Twist()
            return
        axis = lambda index: axes[index] if index < len(axes) else 0.0
        button = lambda index: buttons[index] if index < len(buttons) else 0
        current = [int(bool(button(i))) for i in (0, 1, 3, 4)]
        current.append(int(axis(5) < -0.5))  # LT
        current.append(int(axis(4) < -0.5))  # RT
        start = int(bool(button(11)))
        if self.rearm_required:
            self.target = Twist()
            self.last_buttons = current
            self.last_start = start
            if (abs(axis(6)) < 0.5 and abs(axis(7)) < 0.5
                    and not button(6) and not button(7)):
                self.rearm_required = False
                self.get_logger().info('gamepad neutral; manual control rearmed')
            return
        command = Twist()
        dpad_x = deadzone(axis(6), 0.5)
        dpad_y = deadzone(axis(7), 0.5)
        dpad_length = max(1.0, math.hypot(dpad_x, dpad_y))
        command.linear.x = dpad_y / dpad_length * self.linear_x
        command.linear.y = dpad_x / dpad_length * self.linear_y
        command.angular.z = (int(bool(button(6))) - int(bool(button(7)))) * self.angular_z
        self.target = command if not self.navigation_active else Twist()

        if self.last_start is not None and start and not self.last_start:
            self.voice_enable_pub.publish(Bool(data=True))
            if self.start_sound and os.path.isfile(self.start_sound):
                try:
                    subprocess.Popen(['aplay', '-q', '-D', self.audio_output_device,
                                      self.start_sound],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self.get_logger().info('Start pressed: teleop completion audio played')
                except OSError as error:
                    self.get_logger().error(f'cannot play Start audio: {error}')
        self.last_start = start

        if self.last_buttons is None:
            self.last_buttons = current
            return
        if not self.navigation_active and self.owners_ok():
            for index in range(5):
                if current[index] and not self.last_buttons[index]:
                    self.toggle_pose(index)
                    break  # One arm action per Joy sample; preset takes priority.
            else:
                if current[5] and not self.last_buttons[5]:
                    closed = not self.claw_closed
                    if self.send_pose({10: 50 if closed else 300}, 0.3):
                        self.claw_closed = closed
        self.last_buttons = current

    def toggle_pose(self, index):
        if index == 0:
            updates = A_SECOND_POSE if self.a_second_pose_next else POSES[0]
            if self.send_pose(updates, 1.0):
                self.a_second_pose_next = not self.a_second_pose_next
                self.active_pose = None
            return
        returning = self.active_pose == index
        updates = INIT_POSE if returning else POSES[index]
        if self.send_pose(updates, 1.0):
            self.active_pose = None if returning else index

    def send_pose(self, updates, duration):
        if not self.arm_owners_ok():
            self.get_logger().error('arm command blocked: another /servo_controller publisher exists')
            return False
        message = ServosPosition()
        message.duration = float(duration)
        message.position_unit = 'pulse'
        for servo_id, pulse in updates.items():
            position = ServoPosition()
            position.id = servo_id
            position.position = float(pulse)
            message.position.append(position)
        self.servo_pub.publish(message)
        self.current_pose.update(updates)
        self.get_logger().info(f'arm preset sent: {updates}')
        return True

    def stop(self):
        self.target = Twist()
        if self.manual_locked:
            return
        for _ in range(3):
            self.publish_zero()
            self.publish_lease(False)
            time.sleep(0.02)


def main(args=None):
    stopping = False

    def request_stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = XboxTeleop()
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
