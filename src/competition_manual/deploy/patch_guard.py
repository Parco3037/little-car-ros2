"""One-time, checked patch for the existing RAICOM velocity guard.

The guard remains the sole /controller/cmd_vel publisher. Its original input
continues to reject lateral velocity. A separate manual input accepts lateral
velocity only with a fresh competition_manual lease and no active navigation.
"""

from pathlib import Path


TARGET = Path('/home/ubuntu/ros2_ws/src/raicom_control/raicom_control/cmd_vel_guard.py')
text = TARGET.read_text()
if "'/raicom/manual_cmd'" in text:
    raise SystemExit('manual guard integration already present; no changes made')


def replace_once(old, new):
    global text
    if text.count(old) != 1:
        raise SystemExit(f'expected exactly one anchor, found {text.count(old)}: {old[:80]!r}')
    text = text.replace(old, new)


replace_once(
    'from rclpy.qos import QoSProfile, ReliabilityPolicy',
    'from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy',
)
replace_once(
    "        self.declare_parameter('input_topic', '/raicom/teleop_cmd')",
    "        self.declare_parameter('input_topic', '/raicom/teleop_cmd')\n"
    "        self.declare_parameter('manual_topic', '/raicom/manual_cmd')\n"
    "        self.declare_parameter('manual_max_linear', 0.8)\n"
    "        self.declare_parameter('manual_max_angular', 1.0)",
)
replace_once(
    "        self.stop_latched = False\n        self.stop_subscription = self.create_subscription(\n"
    "            Empty, stop_topic, self._on_stop, 10)",
    "        self.stop_latched = False\n        self.stop_subscription = self.create_subscription(\n"
    "            Empty, stop_topic, self._on_stop, 10)\n"
    "        self.nav_active = False\n"
    "        nav_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,\n"
    "                             durability=DurabilityPolicy.TRANSIENT_LOCAL)\n"
    "        self.nav_subscription = self.create_subscription(\n"
    "            Bool, '/raicom/navigation_active', self._on_nav_active, nav_qos)",
)
replace_once(
    'from std_msgs.msg import Empty, String',
    'from std_msgs.msg import Bool, Empty, String',
)
replace_once(
    "        self.subscription = self.create_subscription(Twist, input_topic, self._on_command, velocity_qos)",
    "        self.subscription = self.create_subscription(Twist, input_topic, self._on_command, velocity_qos)\n"
    "        manual_topic = str(self.get_parameter('manual_topic').value)\n"
    "        self.manual_subscription = self.create_subscription(\n"
    "            Twist, manual_topic, self._on_manual_command, velocity_qos)",
)
replace_once(
    "               'input_topic','output_topic','motion_enable_topic','stop_topic','speed_contract_version'}",
    "               'input_topic','manual_topic','manual_max_linear','manual_max_angular',\n"
    "               'output_topic','motion_enable_topic','stop_topic','speed_contract_version'}",
)
replace_once(
    '        validate_guard_limits(self.max_linear,self.max_angular)',
    '        validate_guard_limits(self.max_linear,self.max_angular)\n'
    "        validate_guard_limits(float(self.get_parameter('manual_max_linear').value),\n"
    "                              float(self.get_parameter('manual_max_angular').value))",
)
replace_once(
    "    def _on_motion_enable(self, msg):\n        self._lease_count",
    "    def _on_motion_enable(self, msg):\n"
    "        previous_mission = self.motion_enable.mission_id\n"
    "        self._lease_count",
)
replace_once(
    "        self.stop_latched = False\n\n    def _on_stop(self, _msg):",
    "        self.stop_latched = False\n"
    "        if self.motion_enable.mission_id != previous_mission:\n"
    "            self.acceleration.reset(now)\n"
    "            self.state.update(ZERO_COMMAND, now)\n\n"
    "    def _on_nav_active(self, message):\n"
    "        self.nav_active = bool(message.data)\n"
    "        if self.nav_active:\n"
    "            now = time.monotonic()\n"
    "            self.acceleration.reset(now)\n"
    "            self.state.update(ZERO_COMMAND, now)\n\n"
    "    def _manual_lease(self):\n"
    "        return (self.motion_enable.valid_at(time.monotonic())\n"
    "                and str(self.motion_enable.mission_id).startswith('competition_manual:'))\n\n"
    "    def _manual_active(self):\n"
    "        return self._manual_lease() and not self.nav_active\n\n"
    "    def _on_stop(self, _msg):",
)
replace_once(
    "    def _on_command(self, msg: Twist) -> None:\n        now = time.monotonic()",
    "    def _on_command(self, msg: Twist) -> None:\n"
    "        if self._manual_lease():\n"
    "            return\n"
    "        self._process_command(msg, allow_lateral=False)\n\n"
    "    def _on_manual_command(self, msg: Twist) -> None:\n"
    "        if self._manual_active():\n"
    "            self._process_command(msg, allow_lateral=True)\n\n"
    "    def _process_command(self, msg: Twist, allow_lateral: bool) -> None:\n"
    "        now = time.monotonic()",
)
replace_once('        if msg.linear.y != 0.0:', '        if msg.linear.y != 0.0 and not allow_lateral:')
replace_once(
    '            (msg.linear.x, msg.linear.y, msg.angular.z), self.max_linear, self.max_angular',
    "            (msg.linear.x, msg.linear.y, msg.angular.z),\n"
    "            float(self.get_parameter('manual_max_linear').value) if allow_lateral else self.max_linear,\n"
    "            float(self.get_parameter('manual_max_angular').value) if allow_lateral else self.max_angular",
)
replace_once(
    '        else:\n            command = self.acceleration.apply(command, now)\n'
    '        self.state.update(command, now)',
    '        elif allow_lateral and command.is_zero:\n'
    '            self.acceleration.reset(now)\n'
    '        else:\n            command = self.acceleration.apply(command, now)\n'
    '        self.state.update(command, now)',
)
replace_once(
    "            'lateral motion disabled (unresolved execution fault)'",
    "            'navigation lateral disabled; manual lease permits lateral'",
)
replace_once(
    '        if (self.stop_latched or not self._motion_enabled()\n'
    '                or self.state.is_stale(now)):',
    '        if (self.stop_latched or not self._motion_enabled()\n'
    '                or (self.nav_active and self._manual_lease())\n'
    '                or self.state.is_stale(now)):',
)
TARGET.write_text(text)
print(f'patched {TARGET}')
