"""Timed forward command after voice locks the gamepad."""

import json
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String


class AutoAdvance(Node):
    def __init__(self):
        super().__init__('competition_auto_advance')
        self.declare_parameter('speed_mps', 0.2)
        self.declare_parameter('duration_sec', 2.7)
        self.speed = float(self.get_parameter('speed_mps').value)
        self.duration = float(self.get_parameter('duration_sec').value)
        if not (0 < self.speed <= 0.2 and 0 < self.duration <= 3.0):
            raise ValueError('auto advance speed or duration outside allowed limits')

        self.velocity_pub = self.create_publisher(Twist, '/raicom/teleop_cmd', 1)
        self.lease_pub = self.create_publisher(String, '/raicom/motion_enable', 1)
        event_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/competition_manual/auto_requested',
                                 self.on_request, event_qos)
        self.create_subscription(Bool, '/raicom/navigation_active',
                                 self.on_navigation, event_qos)
        self.create_subscription(Empty, '/raicom/nav_stop', self.on_stop, 10)
        self.create_subscription(String, '/raicom/guard_status', self.on_guard_status, 10)

        self.state = 'idle'
        self.state_at = time.monotonic()
        self.navigation_active = False
        self.external_stop = False
        self.guard_status = None
        self.guard_at = None
        self.mission_id = 'competition_auto_advance:' + str(uuid.uuid4())
        try:
            with open('/proc/sys/kernel/random/boot_id', encoding='ascii') as source:
                self.boot_id = source.read().strip()
        except OSError:
            self.boot_id = str(uuid.uuid4())
        self.create_timer(0.05, self.tick)

    def on_request(self, message):
        if message.data and self.state == 'idle':
            self.state = 'waiting'
            self.state_at = time.monotonic()
            self.get_logger().info('auto mode: waiting for gamepad control to release')

    def on_navigation(self, message):
        self.navigation_active = bool(message.data)

    def on_stop(self, _message):
        self.external_stop = True

    def on_guard_status(self, message):
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict):
            self.guard_status = status
            self.guard_at = time.monotonic()

    def publish_lease(self, enabled):
        payload = {
            'boot_id': self.boot_id,
            'mission_id': self.mission_id,
            'enabled': bool(enabled),
            'expires_at': time.monotonic() + (0.2 if enabled else 0.0),
        }
        self.lease_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def publish_speed(self, speed):
        command = Twist()
        command.linear.x = speed
        self.velocity_pub.publish(command)

    def owners_ok(self):
        guard = self.get_publishers_info_by_topic('/controller/cmd_vel')
        navigation = self.get_publishers_info_by_topic('/raicom/teleop_cmd')
        manual = self.get_publishers_info_by_topic('/raicom/manual_cmd')
        return (len(guard) == 1 and guard[0].node_name == 'cmd_vel_guard'
                and len(navigation) == 1 and navigation[0].node_name == self.get_name()
                and not manual)

    def stop_motion(self, reason):
        if self.state == 'done':
            return
        self.publish_speed(0.0)
        self.publish_lease(False)
        self.state = 'done'
        self.get_logger().info(f'auto advance stopped: {reason}')

    def tick(self):
        if self.state in ('idle', 'done'):
            return
        now = time.monotonic()
        if self.external_stop or self.navigation_active:
            self.stop_motion('external stop or navigation active')
            return
        if self.state == 'waiting':
            if now - self.state_at > 2.0:
                self.stop_motion('gamepad handoff timeout')
                return
            if not self.owners_ok():
                return
            if (self.guard_status is None or self.guard_at is None
                    or now - self.guard_at > 1.0):
                return
            if self.guard_status.get('stop_latched'):
                self.stop_motion('guard stop latched')
                return
            self.state = 'moving'
            self.state_at = now
            self.get_logger().info('timed forward command started')

        if not self.owners_ok() or self.guard_at is None or now - self.guard_at > 1.0:
            self.stop_motion('controller ownership or guard status lost')
        elif self.guard_status.get('stop_latched'):
            self.stop_motion('guard stop latched')
        elif now - self.state_at >= self.duration:
            self.stop_motion('timed command completed')
        else:
            self.publish_lease(True)
            self.publish_speed(self.speed)

    def destroy_node(self):
        self.publish_speed(0.0)
        self.publish_lease(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoAdvance()
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
