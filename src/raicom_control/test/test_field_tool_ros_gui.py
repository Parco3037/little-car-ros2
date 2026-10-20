"""Native Humble TF buffer and GUI event tests without ROS discovery or motion."""
from copy import deepcopy
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

try:
    import rclpy
except ImportError:
    raise unittest.SkipTest('native ROS/Tk tests run inside MentorPi')

from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import tf2_ros
from raicom_control.field_tool_ros import FieldToolNode
from raicom_control.field_tool_gui import FieldToolApp, MapCanvas, prepare_mapping_arm, occupancy_preview
from raicom_control.field_tool_nav_bridge import NavCmdBridge
from raicom_control.field_tool_nav_gate import NavigationCommandGate


class NativeBridgeTests(unittest.TestCase):
    def test_idle_nav2_zero_stream_does_not_override_manual_commands(self):
        bridge = NS(gate=NavigationCommandGate(), publisher=Mock(),
                    _publish_active=Mock(), was_active=False)
        NavCmdBridge._on_command(bridge, Mock())
        NavCmdBridge._tick(bridge)
        bridge.publisher.publish.assert_not_called()

    def test_expired_context_stops_once_then_releases_manual_channel(self):
        bridge = NS(gate=NavigationCommandGate(), publisher=Mock(),
                    _publish_active=Mock(), was_active=True)
        NavCmdBridge._tick(bridge)
        NavCmdBridge._tick(bridge)
        bridge.publisher.publish.assert_called_once()


class NativeTFTests(unittest.TestCase):
    def make_node(self):
        buffer = tf2_ros.Buffer()
        for parent, child, stamp in [('odom', 'base_footprint', 99.9), ('map', 'odom', 99.), ('map', 'odom', 101.)]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.header.stamp = Time(seconds=stamp).to_msg()
            t.transform.rotation.w = 1.
            buffer.set_transform(t, 'offline-test')
        node = NS(tf_buffer=buffer, get_clock=lambda: NS(now=lambda: Time(seconds=100.)))
        return node

    def test_postdated_amcl_is_queried_at_real_odometry_stamp(self):
        node = self.make_node()
        transform, age = FieldToolNode.fixed_map_transform(node)
        self.assertAlmostEqual(age, .1)
        self.assertEqual(Time.from_msg(transform.header.stamp).nanoseconds, Time(seconds=99.9).nanoseconds)

    def test_stale_odometry_not_hidden_by_future_amcl(self):
        node = self.make_node()
        node.get_clock = lambda: NS(now=lambda: Time(seconds=100.8))
        with self.assertRaisesRegex(RuntimeError, 'odometry TF'):
            FieldToolNode.fixed_map_transform(node)

    def test_missing_map_chain_rejected(self):
        node = self.make_node()
        node.tf_buffer.clear()
        with self.assertRaises(Exception):
            FieldToolNode.fixed_map_transform(node)

    def test_old_zero_command_is_not_current_stop_evidence(self):
        node = NS(zero_hold_s=.5, velocity_at=1., velocity_zero_since=0., velocity_zero=True)
        with patch('raicom_control.field_tool_ros.time.monotonic', return_value=3.):
            self.assertFalse(FieldToolNode.zero_velocity_hold_ok(node))


class GuiEventTests(unittest.TestCase):
    def test_live_map_rows_unknown_cells_and_origin_preserve_coordinates(self):
        from nav_msgs.msg import OccupancyGrid
        message = OccupancyGrid()
        message.info.width, message.info.height, message.info.resolution = 2, 2, .02
        message.info.origin.position.x = 1.0
        message.info.origin.orientation.z = math.sin(.3)
        message.info.origin.orientation.w = math.cos(.3)
        message.data = [0, 100, -1, 50]
        before = list(message.data)
        data, origin, resolution = occupancy_preview(message)
        self.assertEqual(data, b'P5\n2 2\n255\n' + bytes([205, 128, 255, 0]))
        self.assertEqual(list(message.data), before)
        self.assertAlmostEqual(origin[2], .6)
        self.assertEqual((origin[0], resolution), (1., .02))
        message.info.width = 3
        with self.assertRaises(ValueError):
            occupancy_preview(message)

    def test_saved_arm_pose_failure_blocks_startup(self):
        process = Mock()
        process.wait.return_value = 4
        with patch('raicom_control.field_tool_gui.subprocess.Popen', return_value=process) as launch:
            with self.assertRaisesRegex(RuntimeError, '建图未启动'):
                prepare_mapping_arm()
        self.assertIn('/home/ubuntu/raicom_tools/mapping_rest_pose.json', launch.call_args.args[0])
        self.assertIn('--apply', launch.call_args.args[0])

    def test_arm_timeout_requests_graceful_stop_and_blocks_startup(self):
        import subprocess
        process = Mock()
        process.wait.side_effect = [subprocess.TimeoutExpired('arm-rest', 35), 4]
        with patch('raicom_control.field_tool_gui.subprocess.Popen', return_value=process):
            with self.assertRaisesRegex(RuntimeError, '建图未启动'):
                prepare_mapping_arm()
        process.send_signal.assert_called_once()

    def app(self):
        app = FieldToolApp.__new__(FieldToolApp)
        app.node = Mock()
        app.manual_armed = True
        app.held_key = None
        app.navigation_running = False
        app.nav_start_pending = False
        app.mapping_start_pending = False
        app._set_status = Mock()
        return app

    def test_rotated_map_screen_world_roundtrip_including_downscale(self):
        for yaw in (-math.pi/2, .4, math.pi/2):
            for scale in (.25, 1., 3.):
                canvas = NS(origin=[1., -2., yaw], resolution=.02, height=400, scale=scale)
                for x, y in [(1., -2.), (2.1, 3.8), (-.4, 2.3)]:
                    sx, sy = MapCanvas.world_to_screen(canvas, x, y)
                    actual = MapCanvas.screen_to_world(canvas, sx, sy)
                    self.assertAlmostEqual(actual[0], x)
                    self.assertAlmostEqual(actual[1], y)

    def test_typing_coordinates_never_sends_movement(self):
        app = self.app()
        for name in ('Entry', 'TEntry', 'Text', 'TCombobox'):
            widget = Mock()
            widget.winfo_class.return_value = name
            app._key_press(NS(keysym='w', widget=widget))
        app.node.send_key.assert_not_called()

    def test_space_disarms_manual_and_stops_pending_navigation(self):
        app = self.app()
        app.held_key, app.nav_start_pending, app.navigation_running = 'w', True, True
        app._key_press(NS(keysym='space'))
        self.assertFalse(app.manual_armed)
        self.assertFalse(app.nav_start_pending)
        self.assertFalse(app.navigation_running)
        self.assertIsNone(app.held_key)
        app.node.emergency_stop.assert_called_once()

    def test_key_release_sends_only_stop(self):
        app = self.app()
        app._hold_manual('w')
        app._key_release(NS(keysym='w'))
        self.assertIsNone(app.held_key)
        self.assertEqual([c.args[0] for c in app.node.send_key.call_args_list], ['w', ' '])

    def test_cancel_edit_keeps_complete_document_unchanged(self):
        app = self.app()
        app.root = Mock()
        app.route_document = {'order':['p'], 'points':{'p':{'name':'p','label':'old','reference':{'x':0.,'y':0.,'yaw_deg':0.}}},'history':[]}
        before = deepcopy(app.route_document)
        app._selected_route_index = lambda: 0
        with patch('raicom_control.field_tool_gui.simpledialog.askstring', return_value='new'), patch('raicom_control.field_tool_gui.simpledialog.askfloat', return_value=None):
            app.edit_selected_point()
        self.assertEqual(app.route_document, before)

    def test_edit_disabled_during_navigation(self):
        app = self.app()
        app.navigation_running = True
        self.assertFalse(app._can_edit_route())


if __name__ == '__main__':
    unittest.main()
