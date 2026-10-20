from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy'))
from fixed_map_resume import (
    ALIGNMENT_OBSERVE_SEC, BASE_STAMP_MAX_AGE_SEC,
    BASE_STAMP_FUTURE_TOLERANCE_SEC, GUARD_ZERO_MIN_ROWS, GUARD_ZERO_OBSERVE_SEC,
    MAP_IMAGE, MAP_IMAGE_SHA256,
    MAP_YAML, MAP_YAML_SHA256, RecoveryError, assert_prepare_preflight,
    container_pids, guard_zero_result_is_safe, make_session, parse_cmd_vel_csv,
    parse_ros_stamp, parse_throttled, scan_normalizer_launch_value,
    stamp_age_is_fresh, update_guard_dropin,
)


class FixedMapResumePolicyTest(unittest.TestCase):
    def test_unique_session_is_deterministic_and_well_scoped(self):
        session = make_session(datetime(2026, 9, 1, 22, 55, 3), 'a1b2c3')
        self.assertEqual(session, 'local-odom-20260901-225503-a1b2c3')

    def test_guard_dropin_updates_exactly_one_session(self):
        original = "ExecStart=x localization_session:=old-session\n"
        updated = update_guard_dropin(
            original, 'local-odom-20260901-225503-a1b2c3')
        self.assertIn('localization_session:=local-odom-20260901-225503-a1b2c3', updated)
        for bad in ('no token', 'localization_session:=a\nlocalization_session:=b'):
            with self.assertRaises(RecoveryError):
                update_guard_dropin(bad, 'local-odom-20260901-225503-a1b2c3')

    def test_speed_parser_counts_only_real_six_axis_rows(self):
        result = parse_cmd_vel_csv('x,y\n0,0,0,0,0,0\n0.1,0,0,0,0,0\n')
        self.assertEqual(result, {'rows': 2, 'nonzero': 1})

    def test_cold_discovery_gets_slack_without_weakening_zero_gate(self):
        self.assertEqual(GUARD_ZERO_OBSERVE_SEC, 10)
        self.assertEqual(GUARD_ZERO_MIN_ROWS, 40)
        self.assertFalse(guard_zero_result_is_safe({'rows': 35, 'nonzero': 0}))
        self.assertTrue(guard_zero_result_is_safe({'rows': 40, 'nonzero': 0}))
        self.assertFalse(guard_zero_result_is_safe({'rows': 100, 'nonzero': 1}))

    def test_alignment_window_survives_initial_tf_discovery(self):
        # A real 10 s run lost its first sample while map TF appeared and
        # produced only seven valid 1 Hz frames.  Twelve seconds produced 11.
        self.assertEqual(ALIGNMENT_OBSERVE_SEC, 12)

    def test_ros_stamp_parser_and_freshness_bounds_fail_closed(self):
        self.assertEqual(parse_ros_stamp('sec: 10\nnanosec: 250000000\n---\n'), 10.25)
        for text in ('', 'sec: 1\n---\n', 'sec: 1\nnanosec: 1000000000\n---\n'):
            with self.assertRaises(RecoveryError):
                parse_ros_stamp(text)
        self.assertTrue(stamp_age_is_fresh(0.0))
        self.assertTrue(stamp_age_is_fresh(BASE_STAMP_MAX_AGE_SEC))
        self.assertTrue(stamp_age_is_fresh(-BASE_STAMP_FUTURE_TOLERANCE_SEC))
        self.assertFalse(stamp_age_is_fresh(BASE_STAMP_MAX_AGE_SEC + 1e-6))
        self.assertFalse(stamp_age_is_fresh(-BASE_STAMP_FUTURE_TOLERANCE_SEC - 1e-6))

    def test_current_and_historical_power_faults_are_separate(self):
        self.assertEqual(parse_throttled('throttled=0x0')['current_fault_bits'], 0)
        historical = parse_throttled('throttled=0x50000')
        self.assertEqual(historical['current_fault_bits'], 0)
        self.assertNotEqual(historical['historical_fault_bits'], 0)
        self.assertNotEqual(parse_throttled('throttled=0x1')['current_fault_bits'], 0)

    def test_process_inventory_matches_ps_snapshot_not_query_text(self):
        class FakeRunner:
            def run(self, argv, **kwargs):
                self.argv = argv
                return SimpleNamespace(stdout=(
                    '  11   1 /usr/bin/python3 worker.py --session wanted\n'
                    '  12   1 /bin/bash -lc unrelated\n'
                    '  13  11 /usr/bin/python3 worker.py --session wanted\n'
                    '  14   1 /usr/bin/python3 other.py --session wanted\n'))
        runner = FakeRunner()
        self.assertEqual(container_pids(runner, r'^/usr/bin/python3 worker.py '), [11])
        self.assertEqual(runner.argv[-3:], ['ps', '-eo', 'pid=,ppid=,args='])

    def test_source_has_no_actuator_or_navigation_send_path(self):
        text = (Path(__file__).resolve().parents[1] / 'deploy' / 'fixed_map_resume.py').read_text()
        forbidden = ('ros2 action send_goal', 'ros2 topic pub /controller/cmd_vel',
                     'ros2 topic pub /cmd_vel', 'set_position', 'set_gripper')
        for item in forbidden:
            self.assertNotIn(item, text)
        self.assertIn("mode.add_argument('--prepare-startup'", text)
        self.assertIn("if not args.startup_area_confirmed", text)
        self.assertIn("start_scan_normalizer:=' + start_scan_normalizer", text)

    def test_scan_normalizer_starts_after_clean_boot_or_reuses_one(self):
        self.assertEqual(
            scan_normalizer_launch_value({'processes': {'scan_normalizer': []}}), 'true')
        self.assertEqual(
            scan_normalizer_launch_value({'processes': {'scan_normalizer': [40]}}), 'false')
        with self.assertRaisesRegex(RecoveryError, 'zero or one scan_normalizer'):
            scan_normalizer_launch_value({'processes': {'scan_normalizer': [40, 41]}})

    def test_retry_preflight_allows_vendor_ekf_already_stopped(self):
        class FakeRunner:
            def docker_shell(self, command, **kwargs):
                self.command = command
                return SimpleNamespace(returncode=0, stdout='', stderr='')

        status = {
            'services': {
                'start_app_node.service': 'active',
                'raicom_cmd_vel_guard.service': 'active',
            },
            'container_running': True,
            'clock': {'ntp_synchronized': True},
            'throttled': {'current_fault_bits': 0},
            'map_sha256': {
                MAP_YAML: MAP_YAML_SHA256,
                MAP_IMAGE: MAP_IMAGE_SHA256,
            },
            'processes': {
                'controller': [10], 'lidar': [20], 'bringup': [30],
                'vendor_ekf': [], 'runtime': [], 'scan_normalizer': [],
                'map_server': [], 'amcl': [],
            },
        }
        assert_prepare_preflight(FakeRunner(), status)

    def test_retry_preflight_still_rejects_duplicate_vendor_ekf(self):
        class FakeRunner:
            def docker_shell(self, command, **kwargs):
                return SimpleNamespace(returncode=0, stdout='', stderr='')

        status = {
            'services': {
                'start_app_node.service': 'active',
                'raicom_cmd_vel_guard.service': 'active',
            },
            'container_running': True,
            'clock': {'ntp_synchronized': True},
            'throttled': {'current_fault_bits': 0},
            'map_sha256': {
                MAP_YAML: MAP_YAML_SHA256,
                MAP_IMAGE: MAP_IMAGE_SHA256,
            },
            'processes': {
                'controller': [10], 'lidar': [20], 'bringup': [30],
                'vendor_ekf': [40, 41], 'runtime': [], 'scan_normalizer': [],
                'map_server': [], 'amcl': [],
            },
        }
        with self.assertRaisesRegex(RecoveryError, 'zero or one vendor_ekf'):
            assert_prepare_preflight(FakeRunner(), status)

    def test_preflight_rejects_unsynchronised_host_clock(self):
        class FakeRunner:
            def docker_shell(self, command, **kwargs):
                return SimpleNamespace(returncode=0, stdout='', stderr='')

        status = {
            'services': {
                'start_app_node.service': 'active',
                'raicom_cmd_vel_guard.service': 'active',
            },
            'container_running': True,
            'clock': {'ntp_synchronized': False},
            'throttled': {'current_fault_bits': 0},
            'map_sha256': {
                MAP_YAML: MAP_YAML_SHA256,
                MAP_IMAGE: MAP_IMAGE_SHA256,
            },
            'processes': {
                'controller': [10], 'lidar': [20], 'bringup': [30],
                'vendor_ekf': [40], 'runtime': [], 'scan_normalizer': [],
                'map_server': [], 'amcl': [],
            },
        }
        with self.assertRaisesRegex(RecoveryError, 'not NTP-synchronised'):
            assert_prepare_preflight(FakeRunner(), status)


if __name__ == '__main__':
    unittest.main()
