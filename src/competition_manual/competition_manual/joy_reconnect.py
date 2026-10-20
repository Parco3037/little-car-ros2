"""Keep the Xbox Joy driver alive across late connection and radio dropouts."""

import glob
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def matching_joysticks(expected_name):
    devices = []
    for name_path in glob.glob('/sys/class/input/js*/device/name'):
        try:
            with open(name_path, encoding='utf-8') as source:
                name = source.read().strip()
        except OSError:
            continue
        device = '/dev/input/' + name_path.split('/')[4]
        if name == expected_name and os.path.exists(device):
            devices.append(device)
    return devices


class JoyReconnect(Node):
    def __init__(self):
        super().__init__('competition_joy_reconnect')
        self.declare_parameter('input_device_name', 'Xbox Wireless Controller')
        self.declare_parameter('sdl_device_name', 'Xbox One Elite 2 Controller')
        self.input_device_name = str(self.get_parameter('input_device_name').value)
        self.sdl_device_name = str(self.get_parameter('sdl_device_name').value)
        self.joy_executable = os.path.join(get_package_prefix('joy'), 'lib', 'joy', 'joy_node')
        self.child = None
        self.started_at = 0.0
        self.last_joy_at = None
        self.connected = False
        self.last_device_state = None
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(
            Bool, '/competition_manual/joy_connected', state_qos)
        self.state_pub.publish(Bool(data=False))
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f'waiting for {self.input_device_name}; checking every 0.25 s')

    def set_connected(self, connected):
        if connected == self.connected:
            return
        self.connected = connected
        if rclpy.ok(context=self.context):
            self.state_pub.publish(Bool(data=connected))
        self.get_logger().info('gamepad connected' if connected else 'gamepad lost; manual control disabled')

    def on_joy(self, message):
        if self.child is None or self.child.poll() is not None:
            return
        if len(message.axes) < 8 or len(message.buttons) < 12:
            return
        self.last_joy_at = time.monotonic()
        self.set_connected(True)

    def stop_driver(self):
        self.set_connected(False)
        self.last_joy_at = None
        if self.child is None:
            return
        child = self.child
        self.child = None
        if child.poll() is None:
            child.send_signal(signal.SIGTERM)
            try:
                child.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2.0)

    def start_driver(self, now):
        command = [
            self.joy_executable, '--ros-args', '-r', '__node:=xbox_joy',
            '-p', f'device_name:={self.sdl_device_name}',
            '-p', 'deadzone:=0.05', '-p', 'autorepeat_rate:=20.0',
            '-p', 'sticky_buttons:=false',
        ]
        self.started_at = now
        self.last_joy_at = None
        try:
            self.child = subprocess.Popen(command)
        except OSError as error:
            self.get_logger().error(f'cannot start joy_node: {error}')
        else:
            self.get_logger().info('joy_node started; waiting for fresh /joy')

    def tick(self):
        now = time.monotonic()
        devices = matching_joysticks(self.input_device_name)
        state = 'one' if len(devices) == 1 else 'missing' if not devices else 'ambiguous'
        if state != self.last_device_state:
            self.last_device_state = state
            if state == 'missing':
                self.get_logger().warn('gamepad absent; continuing to search')
            elif state == 'ambiguous':
                self.get_logger().error('multiple matching gamepads; waiting for one')
            else:
                self.get_logger().info(f'gamepad device found: {devices[0]}')
        if state != 'one':
            self.stop_driver()
            return
        if self.child is not None and self.child.poll() is not None:
            self.get_logger().warn(f'joy_node exited with code {self.child.returncode}; restarting')
            self.stop_driver()
        if self.child is not None:
            reference = self.last_joy_at if self.last_joy_at is not None else self.started_at
            timeout = 1.0 if self.last_joy_at is not None else 2.0
            if now - reference >= timeout:
                self.get_logger().warn('no fresh /joy; restarting joy_node')
                self.stop_driver()
        if self.child is None and now - self.started_at >= 1.0:
            self.start_driver(now)

    def destroy_node(self):
        self.stop_driver()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyReconnect()
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
