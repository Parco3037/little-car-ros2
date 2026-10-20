"""Capture launch actions only; never start ROS processes or publish commands."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('slam_existing', ROOT/'launch/home_slam.launch.py')
ENTRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTRY)


class ExistingSensorsTest(unittest.TestCase):
    def nodes(self):
        captured = []
        def node(**kwargs):
            captured.append(kwargs)
            # LaunchDescription accepts actual actions; use a harmless declaration.
            return DeclareLaunchArgument('test_'+str(len(captured)), default_value='true')
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(ENTRY.os.path, 'isfile', return_value=True), \
                patch.object(ENTRY, 'Node', side_effect=node):
            launch = ENTRY.generate_launch_description()
        return captured, launch

    def test_only_slam_no_second_normalizer_base_or_guard(self):
        nodes, _ = self.nodes()
        self.assertEqual([n['executable'] for n in nodes],
            ['/home/ubuntu/raicom_tools/slam_online_2_6_8_v1/bin/raicom_online_slam_node'])
        params = nodes[0]['parameters'][1]
        self.assertEqual(params['resolution'], .02)
        self.assertEqual(params['max_laser_range'], 2.)
        self.assertEqual(params['scan_topic'], '/scan')
        self.assertEqual(params['odom_frame'], 'odom')

    def test_refresh_without_extra_nodes_or_sensor_range_change(self):
        nodes, _ = self.nodes()
        params = nodes[0]['parameters'][1]
        self.assertEqual(params['map_update_interval'], .5)
        self.assertEqual(params['minimum_time_interval'], .05)
        self.assertEqual(params['transform_publish_period'], .02)
        self.assertEqual(params['online_max_scan_age'], .5)
        self.assertEqual(params['online_future_tolerance'], .05)
        rviz = (ROOT/'deploy/slam_low_load.rviz').read_text(encoding='utf-8')
        self.assertIn('Frame Rate: 15', rviz)

    def test_heading_library_is_scoped_to_slam_child(self):
        nodes, _ = self.nodes()
        self.assertTrue(nodes[0]['additional_env']['LD_LIBRARY_PATH'].startswith(
            '/home/ubuntu/raicom_tools/slam_toolbox_2_6_8_heading_v1/lib:'))

    def test_missing_heading_library_refuses_silent_legacy_fallback(self):
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(ENTRY.os.path, 'isfile', return_value=False), \
                patch.object(ENTRY, 'Node') as node:
            with self.assertRaisesRegex(RuntimeError, 'heading library is missing'):
                ENTRY.generate_launch_description()
            node.assert_not_called()

    def test_missing_online_executable_refuses_sync_fallback(self):
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(ENTRY.os.path, 'isfile', side_effect=[True, False]), \
                patch.object(ENTRY, 'Node') as node:
            with self.assertRaisesRegex(RuntimeError, 'online SLAM executable is missing'):
                ENTRY.generate_launch_description()
            node.assert_not_called()


if __name__ == '__main__':
    unittest.main()
