"""Real ROS message boundary, fake board; never creates a node or opens serial."""
import ast
import importlib.util
import inspect
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('position_patch', Path(__file__).resolve().parents[1] / 'deploy/patch_bus_servo_position_response.py')
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)
try:
    from ros_robot_controller_msgs.msg import BusServoState, GetBusServoCmd
    from ros_robot_controller_msgs.srv import GetBusServoState
    from ros_robot_controller.ros_robot_controller_node import RosRobotController
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Requires the vendor ROS messages; no hardware used')
class PositionResponseTests(unittest.TestCase):
    def setUp(self):
        source = Path(inspect.getfile(RosRobotController)).read_text(encoding='utf-8')
        fixed = patch.patched(source)
        self.assertEqual(fixed, patch.patched(fixed))
        tree = ast.parse(fixed)
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'get_bus_servo_state')
        scope = {'BusServoState': BusServoState}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<patched-vendor-service>', 'exec'), scope)
        self.method = scope['get_bus_servo_state']
        self.warnings = []
        self.value = [454]
        self.node = SimpleNamespace(board=SimpleNamespace(bus_servo_read_position=lambda _id: self.value), get_logger=lambda: SimpleNamespace(warning=self.warnings.append))

    def query(self):
        req = GetBusServoState.Request(cmd=[GetBusServoCmd(id=3, get_position=1)])
        return self.method(self.node, req, GetBusServoState.Response())

    def test_negative_no_longer_crashes_or_wraps_and_next_request_works(self):
        self.value = [-27]
        reply = self.query()
        self.assertFalse(reply.success)
        self.assertEqual(list(reply.state[0].position), [])
        self.assertIn('[-27]', self.warnings[0])
        self.value = [221]
        self.assertEqual(list(self.query().state[0].position), [221])

    def test_missing_and_malformed_are_not_success(self):
        for value in (None, [], [1, 2], [65536], [1.2], [True]):
            with self.subTest(value=value):
                self.value = value
                reply = self.query()
                self.assertFalse(reply.success)
                self.assertEqual(list(reply.state[0].position), [])

    def test_unsigned_payload_preserved_without_changing_pose_policy(self):
        for value in (0, 221, 1000, 1125, 65535):
            self.value = [value]
            reply = self.query()
            self.assertTrue(reply.success)
            self.assertEqual(list(reply.state[0].position), [value])


if __name__ == '__main__':
    unittest.main()
