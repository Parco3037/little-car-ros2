"""Offline workflow regressions. No discovery, servo or chassis commands."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from raicom_control.field_tool_model import record_point
from raicom_control.field_tool_workspace_model import (
    duplicate_point, execution_identity, point_identity, save_settings, workspace_settings,
)
from raicom_control.field_tool_workspace import WorkspaceMixin, WorkspaceNavigationController
from raicom_control.field_tool_navigation_logic import NavigationTarget


def document():
    d = {'schema': 'raicom_field_route/v2', 'frame': 'map', 'map': {'saved': {'id': 'test'}},
         'order': [], 'points': {}, 'history': []}
    record_point(d, 'A', '园区1', 1., 0., 0.)
    record_point(d, 'B', '园区2', 2., 0., 0.)
    return d


def pose():
    return {'positions': {str(i): 400 for i in (1, 2, 3, 4, 5, 10)},
            'physical_readback_verified': True, 'capture_evidence': 'test/physical_readback',
            'duration_s': 4., 'tolerance_pulse': 10}


class SettingsTests(unittest.TestCase):
    def test_recovery_default_and_persistence_preserve_points_and_speeds(self):
        d = document(); before = deepcopy(d)
        self.assertEqual(workspace_settings(d)['max_recoveries'], 10)
        self.assertEqual(d, before)
        settings = workspace_settings(d); settings.update(max_recoveries=23, speeds={'fine':.5,'near_distance':.1})
        saved = save_settings(d, settings)
        self.assertEqual(workspace_settings(saved)['max_recoveries'], 23)
        self.assertEqual(saved['points'], d['points'])
        self.assertEqual(saved['workspace']['speeds'], settings['speeds'])
        for bad in (-1, 1.5, True, '10', float('inf')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                save_settings(d, {'max_recoveries':bad})

    def test_failed_target_does_not_display_previous_arrival_as_current_error(self):
        nav=NS(state='failed', SUCCEEDED='succeeded', targets=['start','2'],
               current_target=lambda:NS(name='2'), last_error='more than two Nav2 recoveries',
               arrivals=[dict(name='start',position_error_m=.0116,yaw_error_deg=9.,accepted=True,nav_status=4,elapsed_s=2.)])
        app=NS(node=NS(navigation=nav), delayed_start=None, progress_var=Mock(), error_var=Mock(),
               arrival_detail_var=Mock(), route_progress={}, _set_display=lambda var,text:var.set(text))
        WorkspaceMixin._refresh_workspace_metrics(app)
        self.assertIn('2', app.progress_var.set.call_args.args[0])
        self.assertEqual(app.error_var.set.call_args.args[0], '当前失败目标：2')
        self.assertIn('上次停车记录 start', app.arrival_detail_var.set.call_args.args[0])

    def test_duplicate_preserves_original_coordinates_and_history(self):
        d = document(); before = deepcopy(d)
        copied = duplicate_point(d, 'A', 'A_copy')
        self.assertEqual(d, before)
        self.assertEqual(copied['points']['A'], d['points']['A'])
        self.assertEqual(copied['points']['A_copy']['approach'], d['points']['A']['approach'])
        self.assertEqual(copied['history'][:-1], d['history'])
        self.assertEqual(copied['order'], ['A', 'A_copy', 'B'])

    def test_duplicate_rejects_overwrite(self):
        with self.assertRaises(ValueError):
            duplicate_point(document(), 'A', 'B')

    def test_bad_delays_never_reach_execution(self):
        for delay in (-1, 3601, float('nan'), float('inf'), True, '5'):
            with self.subTest(delay=delay), self.assertRaises(ValueError):
                save_settings(document(), {'delay_s': delay})

    def test_pose_is_opt_in_and_coordinate_edit_invalidates_replay(self):
        d = document()
        self.assertFalse(workspace_settings(d)['points'])
        settings = {'points': {'A': {'arm_pose': pose(), 'apply_arm_pose': True,
                                     'pose_fingerprint': point_identity(d['points']['A'])}}}
        d = save_settings(d, settings)
        self.assertTrue(workspace_settings(d)['points']['A']['apply_arm_pose'])
        d['points']['A']['approach']['x'] += .01
        with self.assertRaisesRegex(ValueError, '坐标已变化'):
            workspace_settings(d, require_current_pose=True)
        # Editing/capture stays available to replace the stale pose or disable replay.
        settings = workspace_settings(d)
        settings['points']['A']['apply_arm_pose'] = False
        self.assertFalse(workspace_settings(save_settings(d, settings), require_current_pose=True)['points']['A']['apply_arm_pose'])

    def test_unrecorded_or_guessed_pose_cannot_be_enabled(self):
        d = document()
        for p in (None, {'positions': {'10': 300}}, dict(pose(), physical_readback_verified=False)):
            with self.subTest(p=p), self.assertRaises(ValueError):
                save_settings(d, {'points': {'A': {'apply_arm_pose': True, 'arm_pose': p,
                                                  'pose_fingerprint': point_identity(d['points']['A'])}}})

    def test_settings_change_identity_without_changing_poses(self):
        d = document()
        changed = save_settings(d, {'delay_s': 10, 'destinations': {'A': 'A', 'B': 'B'}})
        self.assertEqual(d['points'], changed['points'])
        self.assertNotEqual(execution_identity(d), execution_identity(changed))


class Var:
    def __init__(self, value=''): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.app = a = WorkspaceMixin()
        a.route_document = document()
        a.map_file = '/tmp/test.yaml'
        a.delayed_start = a.pickup = a.pose_capture = None
        a.arm_io = NS(state='idle', error='', positions={'10': 300}, busy=False,
                      start=Mock(), poll=Mock(), cancel=Mock())
        a.node = NS(external_stop_requested=False, fixed_tf_ok=lambda _: True,
                    sensor_ready=lambda _: True, zero_velocity_hold_ok=lambda: True,
                    navigation=NS(cleanup_pending=False), publish_lease=Mock(), send_key=Mock())
        a._can_edit_route = lambda: not a._workspace_busy()
        a._workspace_snapshot = Mock(return_value='current')
        a._workspace_journal = Mock()
        a._release_manual = Mock()
        a._set_status = Mock()
        a.start_navigation = Mock(side_effect=lambda **_: setattr(a, 'nav_start_pending', True))
        a.navigation_running = a.nav_start_pending = False
        a.destination_vars = {'A': Var('A'), 'B': Var('B')}
        a.destination_combos = {'A': Mock(), 'B': Mock()}
        a.pick_buttons = [Mock(), Mock()]
        a.confirm_pick_button = Mock()
        a.task_var = Var(); a.grip_var = Var(); a.progress_var = Var(); a.status_var = Var()

    def clamped(self):
        self.app.begin_pickup('T')
        self.app.arm_io.state = 'complete'
        self.app._poll_workspace()

    def test_recovery_save_and_dispatch_use_same_setting_without_dispatch_on_save(self):
        a = self.app
        a.recovery_limit_var = Var('17'); a.recovery_status_var = Var()
        a._workspace_write = lambda value:setattr(a, 'route_document', value)
        a.save_navigation_recovery_limit()
        self.assertEqual(a.route_document['workspace']['max_recoveries'], 17)
        a.start_navigation.assert_not_called()
        a.workspace_map_var = Var(); a.navigation_profile_gate = None
        a._prepare_workspace_navigation()
        self.assertEqual(a.node.navigation.max_recoveries, 17)
        self.assertIn('17', a.recovery_status_var.get())

    def test_invalid_or_moving_recovery_setting_never_writes(self):
        a = self.app; a._workspace_write = Mock()
        a.recovery_status_var = Var()
        for text in ('-1','2.5','nan',''):
            a.recovery_limit_var = Var(text); a.save_navigation_recovery_limit()
        a.recovery_limit_var = Var('10'); a.node.zero_velocity_hold_ok = lambda:False
        a.save_navigation_recovery_limit()
        a._workspace_write.assert_not_called()

    def test_grip_never_dispatches_without_human_confirmation(self):
        self.clamped()
        self.assertEqual(self.app.pickup['phase'], 'confirm')
        for _ in range(3): self.app._poll_workspace()
        self.app.start_navigation.assert_not_called()

    def test_confirm_rechecks_servo_and_sends_only_bound_destination_once(self):
        self.clamped()
        self.app.confirm_pickup()
        self.app.arm_io.start.assert_called_with('verify_grip')
        self.app._poll_workspace()
        self.app.confirm_pickup()
        self.app._poll_workspace()
        self.app.start_navigation.assert_called_once_with(target_order=['A'])

    def test_g_maps_to_b_and_unknown_does_not_move(self):
        self.app.begin_pickup('blue')
        self.app.arm_io.start.assert_not_called()
        self.app.begin_pickup('G')
        self.assertEqual(self.app.pickup['destination'], 'B')

    def test_failure_timeout_and_stop_cannot_dispatch_late_completion(self):
        for reason in ('failed', 'timeout', 'stop', 'map_changed'):
            with self.subTest(reason=reason):
                self.setUp(); self.clamped()
                if reason == 'failed': self.app.arm_io.state = 'failed'
                if reason == 'timeout': self.app.pickup['deadline'] = -1.
                if reason == 'stop': self.app.node.external_stop_requested = True
                if reason == 'map_changed':
                    self.app._workspace_snapshot.return_value = 'changed'
                    self.app.confirm_pickup()
                self.app._poll_workspace()
                self.app.arm_io.state = 'complete'
                self.app.confirm_pickup(); self.app._poll_workspace()
                self.app.start_navigation.assert_not_called()
                self.assertIsNone(self.app.pickup)

    def test_duplicate_pickup_is_not_a_second_arm_command(self):
        self.app.begin_pickup('T'); self.app.begin_pickup('G')
        self.app.arm_io.start.assert_called_once_with('grip')

    def test_countdown_cancels_on_stop_and_never_starts_late(self):
        self.app.delayed_start = {'at': -1., 'identity': 'current'}
        self.app.node.external_stop_requested = True
        self.app._poll_workspace()
        self.app.node.external_stop_requested = False
        self.app._poll_workspace()
        self.app.start_navigation.assert_not_called()

    def test_countdown_checks_fresh_pose_and_route_on_expiry(self):
        for reason in ('stale', 'changed', 'ok'):
            with self.subTest(reason=reason):
                self.setUp()
                self.app.delayed_start = {'at': -1., 'identity': 'current'}
                if reason == 'stale': self.app.node.fixed_tf_ok = lambda _: False
                if reason == 'changed': self.app._workspace_snapshot.return_value = 'changed'
                self.app._poll_workspace()
                self.assertEqual(self.app.start_navigation.call_count, int(reason == 'ok'))


class ArrivalTaskTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock(external_stop_requested=False)
        self.node.capture_current_pose.return_value = {'x': 1., 'y': 0., 'yaw_rad': 0.}
        self.node.fixed_tf_ok.return_value = self.node.sensor_ready.return_value = True
        self.node.zero_velocity_hold_ok.return_value = True
        self.workspace = NS(arm_io=Mock())
        self.workspace._arm = lambda: self.workspace.arm_io
        self.c = WorkspaceNavigationController(self.node, self.workspace)
        self.c.targets = [NavigationTarget('A', 'A', 1, 0, 0, .03, 15)]
        self.c.state = self.c.VERIFYING_STOP
        self.c.deadline = 100.
        self.c.started_at = 1.
        self.c.last_nav_status = 4

    def test_default_is_original_completion_and_no_arm(self):
        self.c._verify_stopped_arrival(2.)
        self.assertEqual(self.c.state, self.c.SUCCEEDED)
        self.workspace.arm_io.start.assert_not_called()

    def test_dwell_only_after_both_tolerances_pass_and_without_motion_lease(self):
        self.c.point_tasks = {'A': {'dwell_s': 10}}
        self.c._verify_stopped_arrival(2.)
        self.assertEqual(self.c.state, 'waypoint_task')
        with patch('raicom_control.field_tool_workspace.time.monotonic', return_value=3.): self.c.poll()
        with patch('raicom_control.field_tool_workspace.time.monotonic', return_value=14.): self.c.poll()
        self.assertEqual(self.c.state, self.c.SUCCEEDED)
        self.assertTrue(all(call.args == (False,) for call in self.node.publish_lease.call_args_list))

    def test_outside_tolerance_never_starts_arm_task(self):
        self.c.point_tasks = {'A': {'apply_arm_pose': True, 'arm_pose': pose()}}
        self.node.capture_current_pose.return_value['x'] = 1.031
        self.c._verify_stopped_arrival(2.)
        self.workspace.arm_io.start.assert_not_called()
        self.assertEqual(self.c.state, self.c.FAILED)

    def test_canceled_arrival_task_remains_at_same_resume_point(self):
        self.c.point_tasks = {'A': {'dwell_s': 10}}
        self.c._verify_stopped_arrival(2.)
        self.c.stop()
        self.assertEqual(self.c.index, 0)
        self.assertEqual(self.c.state, self.c.STOPPED)
        self.workspace.arm_io.cancel.assert_called()

    def test_stale_localization_cancels_dwell(self):
        self.c.point_tasks = {'A': {'dwell_s': 10}}
        self.c._verify_stopped_arrival(2.)
        self.node.fixed_tf_ok.return_value = False
        self.c.poll()
        self.assertEqual(self.c.state, self.c.FAILED)


if __name__ == '__main__':
    unittest.main()
