from pathlib import Path
import ast
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raicom_control.nav_control_logic import NavigationCommandGate


class NavigationCommandGateTest(unittest.TestCase):
    def test_fails_closed_until_fresh_manual_suppression(self):
        gate = NavigationCommandGate(.3)
        self.assertFalse(gate.permits_motion(10.0))
        gate.observe_manual_suppressed(True, 10.0)
        self.assertTrue(gate.permits_motion(10.3))
        self.assertFalse(gate.permits_motion(10.301))

    def test_false_ack_closes_gate(self):
        gate = NavigationCommandGate()
        gate.observe_manual_suppressed(True, 1.0)
        gate.observe_manual_suppressed(False, 1.1)
        self.assertFalse(gate.permits_motion(1.1))

    def test_operator_stop_is_irreversibly_latched(self):
        gate = NavigationCommandGate()
        gate.observe_manual_suppressed(True, 1.0)
        self.assertTrue(gate.request_stop())
        self.assertFalse(gate.request_stop())
        gate.observe_manual_suppressed(True, 1.1)
        self.assertFalse(gate.permits_motion(1.1))


class RosWiringSourceTest(unittest.TestCase):
    def test_space_stop_and_cancel_wiring_are_present(self):
        root = Path(__file__).resolve().parents[1]
        bridge = (root/'raicom_control/nav_cmd_bridge.py').read_text()
        keyboard = (root/'raicom_control/keyboard_input.py').read_text()
        self.assertIn("'/raicom/nav_stop'", bridge)
        self.assertIn("'/navigate_to_pose/_action/cancel_goal'", bridge)
        self.assertIn('CancelGoal.Request()', bridge)
        self.assertIn('self.nav_stop_publisher.publish(Empty())', keyboard)
        self.assertIn('if self.navigation_active:', keyboard)

    def test_staging_bridge_launch_is_narrow_and_self_contained(self):
        root = Path(__file__).resolve().parents[1]
        path = root/'launch/nav_cmd_bridge_staging.launch.py'
        source = path.read_text()
        tree = ast.parse(source)
        literals = {
            node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertIn('nav_cmd_bridge', literals)
        self.assertIn('/raicom/nav2_staging/cmd_vel', literals)
        self.assertIn('/raicom/teleop_cmd', literals)
        for forbidden in ('navigation_staging.launch.py', 'NavigateToPose',
                          'home_mission', 'scene_mission', 'arm', 'gripper'):
            self.assertNotIn(forbidden, literals)
        nodes = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name) and node.func.id == 'Node'
        ]
        self.assertEqual(len(nodes), 1)


if __name__ == '__main__':
    unittest.main()
