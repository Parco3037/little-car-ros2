"""Offline configuration contract; no ROS nodes, maps or motion are created."""
from pathlib import Path
import json
import math
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]


class RestoredProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nav = yaml.safe_load(
            (ROOT/'config/nav2_params_home_low_load.yaml').read_text(encoding='utf-8'))
        cls.localization = yaml.safe_load(
            (ROOT/'config/localization_home.yaml').read_text(encoding='utf-8'))
        cls.approach = json.loads(
            (ROOT/'config/navigation_approach_map_01_2cm_20260901_0240.json').read_text(
                encoding='utf-8'))

    def test_both_amcl_profiles_use_sixty_beams_and_two_metres(self):
        for profile in (self.nav, self.localization):
            amcl = profile['amcl']['ros__parameters']
            self.assertEqual(amcl['max_beams'], 60)
            self.assertEqual(amcl['laser_max_range'], 2.0)
            self.assertEqual((amcl['min_particles'], amcl['max_particles']), (500, 3000))

    def test_restore_vendor_refresh_rates(self):
        for name, expected in [('local_costmap', (5.0, 2.0)), ('global_costmap', (1.0, 1.0))]:
            params = self.nav[name][name]['ros__parameters']
            self.assertEqual((params['update_frequency'], params['publish_frequency']), expected)

    def test_keep_range_resolution_safe_scan_and_original_polygon(self):
        for name, layer in [('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')]:
            params = self.nav[name][name]['ros__parameters']
            scan = params[layer]['scan']
            self.assertEqual(scan['raytrace_max_range'], 2.0)
            self.assertEqual(scan['obstacle_max_range'], 2.0)
            self.assertEqual(scan['topic'], '/scan_stable')
            self.assertTrue(scan['marking'])
            self.assertTrue(scan['clearing'])
            self.assertEqual(params['resolution'], .02)
            self.assertEqual(params['footprint_padding'], .05)
            self.assertEqual(params['inflation_layer']['inflation_radius'], .15)
            self.assertNotIn('robot_radius', params)
            self.assertEqual(
                json.loads(params['footprint']),
                self.approach['footprint']['points_xy_m'])

    def test_inflation_covers_padded_inscribed_radius(self):
        points = self.approach['footprint']['points_xy_m']
        padding = self.approach['footprint']['padding_m']
        padded = [
            (x + math.copysign(padding, x) if x else x,
             y + math.copysign(padding, y) if y else y)
            for x, y in points
        ]
        edge_distances = []
        for first, second in zip(padded, padded[1:] + padded[:1]):
            cross = abs(first[0] * second[1] - first[1] * second[0])
            edge_distances.append(
                cross / math.hypot(second[0] - first[0], second[1] - first[1]))
        inscribed_radius = min(edge_distances)
        self.assertAlmostEqual(inscribed_radius, 0.13674992945724732)
        for name in ('local_costmap', 'global_costmap'):
            params = self.nav[name][name]['ros__parameters']
            self.assertGreaterEqual(
                params['inflation_layer']['inflation_radius'], inscribed_radius)

    def test_velocity_limits_match_existing_guard(self):
        smooth = self.nav['velocity_smoother']['ros__parameters']
        self.assertEqual(smooth['max_velocity'], [.25, .25, .35])
        self.assertEqual(smooth['min_velocity'], [-.25, -.25, -.35])
        self.assertEqual(smooth['smoothing_frequency'], 20.)
        self.assertLessEqual(smooth['velocity_timeout'], .15)

    def test_stationary_correction_profiles_match(self):
        for profile in (self.nav, self.localization):
            amcl = profile['amcl']['ros__parameters']
            self.assertEqual(amcl['update_min_a'], .01)
            self.assertEqual(amcl['update_min_d'], .03)
            self.assertEqual(amcl['alpha1'], 4.)
            self.assertEqual(amcl['laser_model_type'], 'likelihood_field_prob')
            self.assertEqual(amcl['sigma_hit'], .05)
            self.assertEqual(amcl['z_hit'], .9)
            self.assertEqual(amcl['z_rand'], .1)

    def test_manual_localization_keeps_mecanum_and_explicit_initial_pose(self):
        for profile in (self.nav, self.localization):
            amcl = profile['amcl']['ros__parameters']
            self.assertEqual(amcl['robot_model_type'], 'nav2_amcl::OmniMotionModel')
            self.assertFalse(amcl['set_initial_pose'])
            self.assertTrue(amcl['always_reset_initial_pose'])
            self.assertNotIn('initial_pose', amcl)

    def test_mecanum_controller_and_staging_isolation(self):
        controller = yaml.safe_load(
            (ROOT/'config/nav2_controller_mecanum.yaml').read_text(encoding='utf-8'))
        p = controller['controller_server']['ros__parameters']
        dwb = p['FollowPath']
        self.assertEqual(dwb['plugin'], 'dwb_core::DWBLocalPlanner')
        self.assertGreater(dwb['max_vel_y'], 0)
        self.assertLess(dwb['min_vel_y'], 0)
        self.assertLessEqual(dwb['max_speed_xy'], .25)
        self.assertLessEqual(dwb['max_vel_theta'], .35)
        self.assertLess(p['min_y_velocity_threshold'], .01)
        self.assertIn('ObstacleFootprint', dwb['critics'])
        self.assertEqual(p['controller_frequency'], 10.)
        self.assertEqual(
            (dwb['vx_samples'], dwb['vy_samples'], dwb['vtheta_samples']),
            (5, 5, 7))
        self.assertEqual(
            dwb['vx_samples'] * dwb['vy_samples'] * dwb['vtheta_samples'],
            175)
        self.assertTrue(all(value % 2 == 1 for value in (
            dwb['vx_samples'], dwb['vy_samples'], dwb['vtheta_samples'])))
        bt = self.nav['bt_navigator']['ros__parameters']
        self.assertEqual(bt['bt_loop_duration'], 100)
        self.assertEqual(bt['default_server_timeout'], 1000)
        self.assertFalse(self.nav['planner_server']['ros__parameters']['use_sim_time'])
        self.assertTrue(self.nav['waypoint_follower']['ros__parameters']['stop_on_failure'])
        import ast
        tree = ast.parse(
            (ROOT/'launch/navigation_staging.launch.py').read_text(encoding='utf-8'))
        source = (ROOT/'launch/navigation_staging.launch.py').read_text(encoding='utf-8')
        literals = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
        self.assertIn('/raicom/nav2_staging/cmd_vel', literals)
        for unsafe in ('/controller/cmd_vel', '/raicom/teleop_cmd', 'nav_cmd_bridge',
                       'home_mission', 'scene_mission', 'scan_normalizer', 'amcl', 'map_server'):
            self.assertNotIn(unsafe, literals)
        self.assertIn('TimerAction', source)
        self.assertIn('latency_sensitive', source)
        self.assertIn('prefix=node_prefix', source)
        self.assertIn("else ['nice -n 10']", source)
        self.assertIn("prefix=['nice -n 10']", source)
        self.assertNotIn("nice -n 19", source)
        self.assertNotIn('taskset -c 2', source)
        self.assertIn("'OPENBLAS_NUM_THREADS': '1'", source)
        self.assertIn("'OMP_NUM_THREADS': '1'", source)
        self.assertIn('period=12.0', source)

    def test_bond_compatibility_is_scoped_and_verified(self):
        import hashlib
        import ast
        library = ROOT/'compat/bondcpp/libbondcpp.so'
        digest = hashlib.sha256(library.read_bytes()).hexdigest()
        source = (ROOT/'launch/navigation_staging.launch.py').read_text(encoding='utf-8')
        self.assertIn(digest, source)
        self.assertEqual(digest, 'd110777939d51480fa4529c68c3b8f220c2b27d0a39cae48a00f052e558e539c')
        tree = ast.parse(source)
        nodes = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'Node']
        self.assertEqual(len(nodes), 2)
        self.assertTrue(all(any(k.arg == 'additional_env' for k in n.keywords) for n in nodes))
        self.assertNotIn('bond_timeout', source)  # Keep the default heartbeat safety.
        self.assertIn('compat', (ROOT/'setup.py').read_text(encoding='utf-8'))

    def test_runtime_audit_reads_polygon_not_removed_radius(self):
        source = (ROOT/'deploy/verify_navigation_staging.py').read_text(encoding='utf-8')
        self.assertEqual(source.count("['resolution', 'footprint_padding', 'footprint']"), 2)
        self.assertNotIn("['resolution', 'footprint_padding', 'robot_radius']", source)
        self.assertIn('def call_with_retry(', source)
        self.assertIn("'lifecycle:'+name", source)
        self.assertIn("'parameters:'+n", source)
        self.assertNotIn('client.call_async(GetState.Request())', source)
        for name in ('FollowPath.vx_samples', 'FollowPath.vy_samples',
                     'FollowPath.vtheta_samples', 'FollowPath.sim_time'):
            self.assertIn(name, source)
        self.assertIn("'expected_samples': [5, 5, 7]", source)
        self.assertIn("'maximum_allowed': 200", source)


if __name__ == '__main__':
    unittest.main()
