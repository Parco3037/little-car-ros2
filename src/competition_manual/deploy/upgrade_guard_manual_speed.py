"""One-time source upgrade: give the manual input separate speed ceilings."""

from pathlib import Path

target = Path('/home/ubuntu/ros2_ws/src/raicom_control/raicom_control/cmd_vel_guard.py')
source = target.read_text()
if "'manual_max_linear'" in source:
    print('manual speed limits already installed')
    raise SystemExit(0)


def replace_once(old, new):
    global source
    if source.count(old) != 1:
        raise SystemExit(f'expected one guard anchor, found {source.count(old)}: {old[:80]!r}')
    source = source.replace(old, new)


replace_once(
    "        self.declare_parameter('manual_topic', '/raicom/manual_cmd')",
    "        self.declare_parameter('manual_topic', '/raicom/manual_cmd')\n"
    "        self.declare_parameter('manual_max_linear', 0.8)\n"
    "        self.declare_parameter('manual_max_angular', 1.0)",
)
replace_once(
    '        validate_guard_limits(self.max_linear,self.max_angular)',
    '        validate_guard_limits(self.max_linear,self.max_angular)\n'
    "        validate_guard_limits(float(self.get_parameter('manual_max_linear').value),\n"
    "                              float(self.get_parameter('manual_max_angular').value))",
)
replace_once(
    "               'input_topic','manual_topic','output_topic','motion_enable_topic','stop_topic','speed_contract_version'}",
    "               'input_topic','manual_topic','manual_max_linear','manual_max_angular',\n"
    "               'output_topic','motion_enable_topic','stop_topic','speed_contract_version'}",
)
replace_once(
    '            (msg.linear.x, msg.linear.y, msg.angular.z), self.max_linear, self.max_angular',
    "            (msg.linear.x, msg.linear.y, msg.angular.z),\n"
    "            float(self.get_parameter('manual_max_linear').value) if allow_lateral else self.max_linear,\n"
    "            float(self.get_parameter('manual_max_angular').value) if allow_lateral else self.max_angular",
)
target.write_text(source)
print(f'upgraded {target}')
