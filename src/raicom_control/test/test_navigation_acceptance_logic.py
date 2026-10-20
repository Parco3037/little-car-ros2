"""Offline tests: no ROS graph, goal, velocity or actuator contact."""

import argparse
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from raicom_control.navigation_acceptance_logic import (
    LegProgressMonitor,
    graph_preflight_errors,
    load_leg_spec,
    load_resume_reference,
    static_pose_collision_cells,
    source_pose_errors,
)


ROOT = Path(__file__).resolve().parents[1]
MAP_FILE = ROOT/'maps/map_01_2cm_20260901_0240.yaml'
if not MAP_FILE.exists():
    MAP_FILE = Path('/home/ubuntu/ros2_ws/src/slam/maps/map_01_2cm_20260901_0240.yaml')


class LegProgressMonitorTests(unittest.TestCase):
    def test_recovery_limit_is_fail_closed(self):
        monitor = LegProgressMonitor(10.0, max_recoveries=2)
        self.assertIsNone(monitor.observe(10.1, 1.0, 2))
        self.assertEqual(monitor.observe(10.2, .9, 3), 'too_many_recoveries:3>2')

    def test_only_real_progress_resets_deadline(self):
        monitor = LegProgressMonitor(0.0, progress_timeout_sec=8.0, min_progress_m=.02)
        monitor.observe(.1, 1.00, 0)
        monitor.observe(7.0, .99, 0)
        self.assertEqual(monitor.check(8.2), 'no_progress:8.100s')
        monitor = LegProgressMonitor(0.0, progress_timeout_sec=8.0, min_progress_m=.02)
        monitor.observe(.1, 1.00, 0)
        self.assertIsNone(monitor.observe(7.0, .97, 0))
        self.assertIsNone(monitor.check(8.2))

    def test_stale_feedback_and_total_timeout(self):
        monitor = LegProgressMonitor(5.0, feedback_timeout_sec=2.0, leg_timeout_sec=60.0)
        self.assertEqual(monitor.check(7.01), 'feedback_stale:2.010s')
        monitor = LegProgressMonitor(5.0, feedback_timeout_sec=100.0, leg_timeout_sec=60.0)
        self.assertEqual(monitor.check(65.01), 'leg_timeout:60.010s')

    def test_near_goal_rotation_is_exempt_from_progress_deadline(self):
        monitor = LegProgressMonitor(0.0, progress_timeout_sec=2.0,
                                     feedback_timeout_sec=10.0,
                                     progress_exempt_distance_m=.08)
        monitor.observe(.1, .05, 0)
        self.assertIsNone(monitor.check(5.0))

    def test_invalid_feedback_rejected(self):
        monitor = LegProgressMonitor(0.0)
        self.assertEqual(monitor.observe(.1, math.nan, 0), 'invalid_feedback')
        self.assertEqual(monitor.observe(.1, 1.0, -1), 'invalid_feedback')


class LegConfigurationTests(unittest.TestCase):
    def test_real_route_is_bound_to_real_preserved_map(self):
        spec = load_leg_spec(
            ROOT/'config/route_map_01_2cm_20260901_0240.json',
            MAP_FILE,
            'start', 'recognition')
        self.assertEqual(spec.route_sha256,
                         '35ca92aa31017e67a8540b160c31a393fa84f1c8d2d57ad5e6d38edb0c6dbcbc')
        self.assertEqual(spec.target_name, 'recognition')

    def test_real_approach_profile_is_bound_and_all_five_goals_are_clear(self):
        approach = ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json'
        spec = load_leg_spec(
            ROOT/'config/route_map_01_2cm_20260901_0240.json',
            MAP_FILE, 'start', 'recognition', approach)
        self.assertEqual(spec.target['label'], '识别区导航接近点')
        self.assertEqual(spec.source_reference_kind, 'work_pose')
        self.assertAlmostEqual(spec.source['x'], 0.0010750674640161717)
        self.assertAlmostEqual(spec.target['x'], 0.17835761355675552)
        self.assertAlmostEqual(spec.target['y'], 0.6271747711513235)
        self.assertTrue(spec.target['camera_observation_revalidation_required'])
        self.assertEqual(set(spec.static_checks),
                         {'start', 'recognition', 'pickup', 'park_1', 'park_2'})
        self.assertTrue(all(row['blocked_cell_intersections'] == 0
                            for row in spec.static_checks.values()))
        self.assertEqual(set(spec.rotation_checks), {
            'start->recognition', 'recognition->pickup',
            'pickup->park_1', 'pickup->park_2',
            'park_1->pickup', 'park_1->start', 'park_2->start'})
        self.assertTrue(all(row['blocked_sample_count'] == 0
                            and row['blocked_cell_intersections'] == 0
                            for row in spec.rotation_checks.values()))
        self.assertEqual(
            spec.rotation_checks['start->recognition']['sample_count'], 19)
        self.assertIsNotNone(spec.approach_sha256)
        self.assertEqual(spec.work_target['label'], '识别区')
        next_leg = load_leg_spec(
            ROOT/'config/route_map_01_2cm_20260901_0240.json',
            MAP_FILE, 'recognition', 'pickup', approach)
        self.assertEqual(next_leg.source_reference_kind, 'approach_pose')
        self.assertAlmostEqual(next_leg.source['x'], spec.target['x'])
        self.assertEqual(next_leg.target['offset_from_work_pose_m'], [-0.12, 0.0])
        self.assertAlmostEqual(next_leg.target['x'], 1.4598703698404766)
        self.assertTrue(next_leg.target['live_dynamic_costmap_revalidation_required'])

    def test_old_recognition_offset_is_rejected_by_terminal_rotation_sweep(self):
        route = ROOT/'config/route_map_01_2cm_20260901_0240.json'
        source = json.loads(
            (ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json').read_text(
                encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory)/'approach.json'
            old = deepcopy(source)
            old['points']['recognition'].update({
                'x': 0.11835761355675552,
                'y': 0.6371747711513235,
                'offset_from_work_pose_m': [0.0, -0.03],
            })
            candidate.write_text(json.dumps(old), encoding='utf-8')
            with self.assertRaisesRegex(
                    ValueError, 'terminal rotation start->recognition'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)

    def test_static_only_or_incomplete_sweep_contract_fails_closed(self):
        route = ROOT/'config/route_map_01_2cm_20260901_0240.json'
        source = json.loads(
            (ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json').read_text(
                encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory)/'approach.json'
            legacy = deepcopy(source)
            legacy['schema'] = 'raicom_navigation_approach/v1'
            candidate.write_text(json.dumps(legacy), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'schema'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)
            incomplete = deepcopy(source)
            incomplete['mission_legs'].pop()
            candidate.write_text(json.dumps(incomplete), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'leg set'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)
            wrong_start = deepcopy(source)
            wrong_start['mission_legs'][0]['source_reference'] = 'approach_pose'
            candidate.write_text(json.dumps(wrong_start), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'Initial start leg'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)

    def test_unshifted_work_points_explain_the_collision_failure(self):
        route = json.loads(
            (ROOT/'config/route_map_01_2cm_20260901_0240.json').read_text(
                encoding='utf-8'))
        approach = json.loads(
            (ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json').read_text(
                encoding='utf-8'))
        collisions = {
            name: len(static_pose_collision_cells(MAP_FILE, approach['footprint'], pose))
            for name, pose in route['points'].items()
        }
        self.assertGreater(collisions['start'], 0)
        self.assertGreater(collisions['recognition'], 0)
        self.assertGreater(collisions['park_1'], 0)
        self.assertEqual(collisions['pickup'], 0)
        self.assertEqual(collisions['park_2'], 0)

    def test_nonfinite_approach_or_footprint_fails_closed(self):
        route = ROOT/'config/route_map_01_2cm_20260901_0240.json'
        source = json.loads(
            (ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json').read_text(
                encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            candidate = Path(directory)/'approach.json'
            bad_pose = deepcopy(source)
            bad_pose['points']['recognition']['x'] = math.nan
            candidate.write_text(json.dumps(bad_pose), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'offset is invalid'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)
            excessive_offset = deepcopy(source)
            excessive_offset['points']['pickup'].update({
                'x': 1.3698703698404766,
                'offset_from_work_pose_m': [-0.21, 0.0],
            })
            candidate.write_text(json.dumps(excessive_offset), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'offset is invalid'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)
            bad_footprint = deepcopy(source)
            bad_footprint['footprint']['points_xy_m'][0][0] = math.nan
            candidate.write_text(json.dumps(bad_footprint), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'non-finite'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)
            bad_step = deepcopy(source)
            bad_step['terminal_rotation_sweep_step_rad'] = math.radians(5.01)
            candidate.write_text(json.dumps(bad_step), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'sweep step'):
                load_leg_spec(route, MAP_FILE, 'start', 'recognition', candidate)

    def test_resume_reference_is_bound_to_failed_leg_and_stop_latch(self):
        approach = ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json'
        spec = load_leg_spec(
            ROOT/'config/route_map_01_2cm_20260901_0240.json',
            MAP_FILE, 'start', 'recognition', approach)
        document = {
            'schema': 'raicom_navigation_acceptance/v1',
            'phase': 'terminal', 'outcome': 'NOT_SUCCEEDED',
            'stop_latch_published': True,
            'leg': {'source': 'start', 'target': 'recognition',
                    'route_sha256': spec.route_sha256,
                    'approach_sha256': spec.approach_sha256},
            'feedback': [{'pose': {'x': .3, 'y': .4, 'z': math.sin(-.5),
                                   'w': math.cos(-.5)}}],
            'finished_at_utc': '2026-09-01T20:53:23+00:00',
            'abort_reason': 'too_many_recoveries:4>2',
        }
        with tempfile.TemporaryDirectory() as directory:
            evidence = Path(directory)/'resume.json'
            evidence.write_text(json.dumps(document), encoding='utf-8')
            reference = load_resume_reference(evidence, spec)
            self.assertAlmostEqual(reference['yaw_rad'], -1.0)
            self.assertEqual(reference['prior_outcome'], 'NOT_SUCCEEDED')
            document['stop_latch_published'] = False
            evidence.write_text(json.dumps(document), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'stop latch'):
                load_resume_reference(evidence, spec)
            document['stop_latch_published'] = True
            document['leg']['approach_sha256'] = '0' * 64
            evidence.write_text(json.dumps(document), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'navigation approach'):
                load_resume_reference(evidence, spec)

    def test_changed_map_binding_and_invalid_leg_rejected(self):
        route = ROOT/'config/route_map_01_2cm_20260901_0240.json'
        with self.assertRaisesRegex(ValueError, 'must differ'):
            load_leg_spec(route, MAP_FILE, 'start', 'start')
        with self.assertRaisesRegex(ValueError, 'not recorded'):
            load_leg_spec(route, MAP_FILE, 'start', 'missing')

    def test_source_pose_and_graph_gate(self):
        source = {'x': 1.0, 'y': 2.0, 'yaw_rad': math.pi - .1}
        position, heading = source_pose_errors(1.03, 1.96, -math.pi + .1, source)
        self.assertAlmostEqual(position, .05)
        self.assertAlmostEqual(heading, .2)
        good = {
            'controller_publishers': ['cmd_vel_guard'],
            'staging_publishers': ['velocity_smoother'],
            'staging_subscribers': ['nav_cmd_bridge'],
            'localization_publishers': ['continuous_fusion_runtime'],
            'map_publishers': ['map_server'],
        }
        self.assertEqual(graph_preflight_errors(good), [])
        broken = deepcopy(good)
        broken['controller_publishers'].append('unsafe')
        self.assertIn('controller_publishers', graph_preflight_errors(broken)[0])

    def test_cli_authorization_is_explicit_and_source_has_only_stop_publisher(self):
        path = ROOT/'raicom_control/navigation_acceptance_leg.py'
        source = path.read_text(encoding='utf-8')
        tree = ast.parse(source)
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
        publishers = [node for node in calls if isinstance(node.func, ast.Attribute)
                      and node.func.attr == 'create_publisher']
        self.assertEqual(len(publishers), 1)
        self.assertEqual(ast.literal_eval(publishers[0].args[1]), '/raicom/nav_stop')
        self.assertIn('ActionClient', source)
        self.assertIn('send_goal_async', source)
        self.assertIn('cancel_goal_async', source)
        self.assertIn('goal_response_timeout', source)
        self.assertIn('late_goal_accepted_after_abort', source)
        self.assertIn("lookup_transform(\n                'map', 'base_footprint'", source)
        self.assertIn("--preflight-only", source)
        subscriptions = [node for node in calls if isinstance(node.func, ast.Attribute)
                         and node.func.attr == 'create_subscription']
        literal_subscription_topics = {
            ast.literal_eval(node.args[1]) for node in subscriptions
            if isinstance(node.args[1], ast.Constant)
        }
        self.assertNotIn('/amcl_pose', literal_subscription_topics)
        self.assertNotIn('Twist', source)
        self.assertNotIn('ServoPosition', source)
        method = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                      and node.name == 'parse_args')
        namespace = {'argparse': argparse, 'math': math}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<args>', 'exec'), namespace)
        with self.assertRaises(SystemExit):
            namespace['parse_args']([])
        parsed, _ = namespace['parse_args']([
            '--route-file', 'r', '--map-file', 'm', '--approach-file', 'a',
            '--source', 'start',
            '--target', 'recognition', '--evidence-file', 'e'])
        self.assertFalse(parsed.operator_authorized)
        self.assertFalse(parsed.preflight_only)


if __name__ == '__main__':
    unittest.main()
