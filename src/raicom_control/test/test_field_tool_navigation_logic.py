"""Offline action races and real bridge TTL protocol; no ROS or actuators."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
import math
import unittest
from unittest.mock import Mock, patch
from raicom_control.field_tool_navigation_logic import NavigationController, NavigationTarget, arrival_errors, POSITION_TOLERANCE_M, YAW_TOLERANCE_DEG, repeat_target_order
from raicom_control.field_tool_nav_gate import NavigationCommandGate


def done(value):
    f = Future()
    f.set_result(value)
    return f


class RepeatOrderTests(unittest.TestCase):
    def setUp(self):
        self.points = {
            '111': {'approach': {'x': 0., 'y': 0., 'yaw_rad': 0.}},
            '222': {'approach': {'x': 1., 'y': 0., 'yaw_rad': math.pi}},
        }

    def test_five_rounds_end_at_actual_start_and_preserve_points(self):
        from copy import deepcopy
        before = deepcopy(self.points)
        for source, other in [('111', '222'), ('222', '111')]:
            seq = repeat_target_order(self.points, '111', '222', 5, self.points[source]['approach'])
            self.assertEqual(seq, [other, source] * 5)
            self.assertEqual(seq.count(source), 5)
        self.assertEqual(before, self.points)

    def test_outside_start_position_or_heading_is_not_an_approach_leg(self):
        for pose in [{'x': .031, 'y': 0., 'yaw_rad': 0.},
                     {'x': .01, 'y': 0., 'yaw_rad': math.radians(10.1)},
                     {'x': .5, 'y': 0., 'yaw_rad': 0.}]:
            with self.assertRaises(ValueError):
                repeat_target_order(self.points, '111', '222', 5, pose)

    def test_invalid_pair_and_rounds_rejected(self):
        for a, b, count in [('111', '111', 5), ('111', 'missing', 5),
                            ('111', '222', 0), ('111', '222', 21), ('111', '222', True)]:
            with self.assertRaises(ValueError):
                repeat_target_order(self.points, a, b, count, self.points['111']['approach'])


class NavigationLogicTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.0
        self.clock = patch('raicom_control.field_tool_navigation_logic.time.monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = Mock()
        self.node.fixed_tf_ok.return_value = True
        self.node.sensor_ready.return_value = True
        self.node.zero_velocity_hold_ok.return_value = True
        self.node.navigation_ready.return_value = True
        self.node.capture_current_pose.return_value = {'x':1., 'y':0., 'yaw_rad':0.}
        self.node.set_controller_tolerances.return_value = done(NS(results=[NS(successful=True)] * 4))
        self.send, self.result = Future(), Future()
        self.handle = Mock(accepted=True)
        self.handle.get_result_async.return_value = self.result
        self.node.navigate_client.send_goal_async.return_value = self.send
        self.controller = NavigationController(self.node)

    def target(self, name='point'):
        return NavigationTarget(name, name, 1., 0., 0., .02, 10.)

    def start(self, count=1):
        self.assertTrue(self.controller.start([self.target(str(i)) for i in range(count)], 'mission'))
        self.controller.poll()

    def accept(self):
        self.send.set_result(self.handle)
        self.controller.poll()

    def test_acceptance_sends_once_and_tolerances_acknowledged_first(self):
        params = Future()
        self.node.set_controller_tolerances.return_value = params
        self.start()
        self.controller.poll()
        self.node.navigate_client.send_goal_async.assert_not_called()
        params.set_result(NS(results=[NS(successful=True)] * 4))
        self.controller.poll()
        self.controller.poll()
        self.node.navigate_client.send_goal_async.assert_called_once()
        self.node.publish_lease.assert_not_called()

    def test_lease_context_remains_valid_across_original_half_second_expiry(self):
        gate = NavigationCommandGate(expected_boot_id='boot')
        self.node.publish_goal_context.side_effect = lambda goal, active, mission_id: gate.receive_goal_context(
            dict(boot_id='boot', mission_id=mission_id, goal_id=goal, active=active, expires_at=self.now + (.5 if active else 0)), self.now)
        self.node.publish_manual_suppressed.side_effect = lambda value: gate.observe_manual_suppressed(value, self.now)
        self.node.publish_nav_stop.side_effect = gate.request_stop
        self.start()
        self.accept()
        for _ in range(30):
            self.now += .1
            self.controller.poll()
            self.assertTrue(gate.permits_motion(self.now))
        self.controller.stop()
        self.assertFalse(gate.permits_motion(self.now))

    def test_stop_before_acceptance_cancels_late_goal_and_blocks_restart(self):
        self.start()
        self.controller.stop()
        self.assertFalse(self.controller.start([self.target()], 'new'))
        self.send.set_result(self.handle)
        self.controller.poll()
        self.handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.controller.start([self.target()], 'new'))
        self.result.set_result(NS(status=5))
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.STOPPED)
        self.node.navigate_client.send_goal_async.assert_called_once()
        self.assertTrue(self.controller.start([self.target()], 'explicit-restart'))

    def test_stop_with_late_success_does_not_advance_queue(self):
        self.start(2)
        self.accept()
        self.controller.stop()
        self.result.set_result(NS(status=4))
        self.controller.poll()
        self.assertEqual(self.controller.index, 0)
        self.assertEqual(self.controller.state, self.controller.STOPPED)

    def test_success_advances_only_after_zero_hold(self):
        self.start(2)
        self.accept()
        self.result.set_result(NS(status=4))
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.VERIFYING_STOP)
        self.now += .6
        self.node.zero_velocity_hold_ok.return_value = False
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.VERIFYING_STOP)
        self.node.zero_velocity_hold_ok.return_value = True
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.ZERO_HOLD)
        self.now += .6
        self.controller.poll()
        self.send, self.result = Future(), Future()
        self.handle = Mock(accepted=True)
        self.handle.get_result_async.return_value = self.result
        self.node.navigate_client.send_goal_async.return_value = self.send
        self.controller.poll()
        self.accept()
        self.result.set_result(NS(status=4))
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.VERIFYING_STOP)
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.SUCCEEDED)

    def test_rejected_parameters_send_no_goal(self):
        self.node.set_controller_tolerances.return_value = done(NS(results=[NS(successful=False)] * 4))
        self.start()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.node.navigate_client.send_goal_async.assert_not_called()

    def test_first_rejection_retries_same_target_once_only_after_active_and_zero_hold(self):
        self.start()
        self.send.set_result(Mock(accepted=False)); self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.ZERO_HOLD)
        self.assertEqual(self.controller.index, 0)
        self.node.reset_navigation_readiness.assert_called_once()
        self.node.navigation_ready.return_value = False
        self.now += 1.1; self.controller.poll(); self.controller.poll()
        self.node.navigate_client.send_goal_async.assert_called_once()
        self.node.publish_lease.assert_called_once_with(False)
        second = Future(); self.node.navigate_client.send_goal_async.return_value = second
        self.node.navigation_ready.return_value = True; self.controller.poll()
        second.set_result(Mock(accepted=False)); self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertEqual(len(self.controller.dispatch_receipts), 2)

    def test_phased_stopped_overshoot_retries_same_goal_but_never_advances_unaccepted_point(self):
        self.controller.arrival_retry_limit = 2
        self.start(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.capture_current_pose.return_value = {'x':1.025,'y':0.,'yaw_rad':0.}
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.ZERO_HOLD)
        self.assertEqual(self.controller.index, 0)
        self.assertEqual(self.controller.arrival_retries, 1)
        self.assertFalse(self.controller.arrivals[-1]['accepted'])
        self.controller.stop(); self.now += 1.; self.controller.poll()
        self.node.navigate_client.send_goal_async.assert_called_once()

    def test_sensor_loss_revokes_and_cancels(self):
        self.start()
        self.accept()
        self.node.sensor_ready.return_value = False
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.node.publish_lease.assert_called_with(False)
        self.handle.cancel_goal_async.assert_called_once()

    def test_acceptance_timeout_still_cancels_later(self):
        self.start()
        self.now += 6
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.send.set_result(self.handle)
        self.controller.poll()
        self.handle.cancel_goal_async.assert_called_once()

    def test_old_feedback_ignored(self):
        self.start()
        callback = self.node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
        self.controller.stop()
        callback(NS(feedback=NS(distance_remaining=9, number_of_recoveries=5)))
        self.assertEqual(self.controller.feedback['recoveries'], 0)

    def test_default_ten_recoveries_continue_then_eleven_cancel_same_goal(self):
        self.assertEqual(self.controller.max_recoveries, 10)
        self.start(); self.accept()
        feedback = self.node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
        for count in (2, 3, 6, 10):
            feedback(NS(feedback=NS(distance_remaining=.5, number_of_recoveries=count)))
            self.controller.poll()
            self.assertEqual(self.controller.state, self.controller.ACTIVE)
            self.handle.cancel_goal_async.assert_not_called()
        feedback(NS(feedback=NS(distance_remaining=.5, number_of_recoveries=11)))
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertIn('11 > 10', self.controller.last_error)
        self.assertEqual(self.controller.index, 0)
        self.handle.cancel_goal_async.assert_called_once()
        self.node.publish_lease.assert_called_with(False)

    def test_custom_recovery_limit_frozen_for_run_and_used_after_restart(self):
        self.controller.max_recoveries = 20
        self.start(); self.accept()
        self.controller.max_recoveries = 1
        self.controller._on_feedback(NS(feedback=NS(distance_remaining=.5, number_of_recoveries=20)), self.controller.goal_id)
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.ACTIVE)
        self.controller.stop(); self.result.set_result(NS(status=5)); self.controller.poll()
        self.controller.start([self.target()], 'new-run')
        self.assertEqual(self.controller.active_max_recoveries, 1)

    def test_zero_limit_stops_on_first_recovery(self):
        self.controller.max_recoveries = 0
        self.start(); self.accept()
        self.controller._on_feedback(NS(feedback=NS(distance_remaining=.5, number_of_recoveries=1)), self.controller.goal_id)
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertIn('1 > 0', self.controller.last_error)

    def test_action_transport_exception_fails_closed(self):
        self.node.navigate_client.send_goal_async.side_effect = RuntimeError('transport')
        self.start()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.node.publish_lease.assert_called_with(False)

    def test_external_stop_cannot_advance_an_active_queue(self):
        self.start(2)
        self.accept()
        self.node.external_stop_requested = True
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.STOPPED)
        self.assertFalse(self.node.external_stop_requested)

    def test_service_discovery_without_active_lifecycle_sends_no_goal(self):
        self.node.navigation_ready.return_value = False
        self.start()
        self.node.navigate_client.send_goal_async.assert_not_called()
        self.node.publish_lease.assert_not_called()

    def test_late_position_drift_rejects_nav2_success_despite_good_angle(self):
        self.start(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.capture_current_pose.return_value = {'x':1.021, 'y':0., 'yaw_rad':0.}
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertEqual(self.controller.index, 0)
        self.assertFalse(self.controller.arrivals[-1]['accepted'])

    def test_good_position_with_angle_over_10_cannot_advance(self):
        import math
        self.start(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.capture_current_pose.return_value = {'x':1., 'y':0., 'yaw_rad':math.radians(10.1)}
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertEqual(self.controller.index, 0)

    def test_stop_is_required_even_after_last_goal_success(self):
        self.start(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.zero_velocity_hold_ok.return_value = False
        self.now += 5.1
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.node.capture_current_pose.assert_not_called()

    def test_angle_error_uses_shortest_difference_across_180(self):
        import math
        target=NavigationTarget('wrap','wrap',0.,0.,math.radians(175),.02,10.)
        position, angle=arrival_errors({'x':.02,'y':0.,'yaw_rad':math.radians(-175)}, target)
        self.assertEqual(position,.02)
        self.assertAlmostEqual(angle,10.)

    def test_two_cm_ten_deg_acceptance_requires_stopped_simultaneous_pose(self):
        import math
        target = NavigationTarget('point', 'point', 1., 0., 0., POSITION_TOLERANCE_M, YAW_TOLERANCE_DEG)
        self.assertTrue(self.controller.start([target], 'two-cm-ten-deg'))
        self.controller.poll(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.capture_current_pose.return_value = {'x':1.019, 'y':0., 'yaw_rad':math.radians(9.9)}
        self.node.zero_velocity_hold_ok.return_value = False
        self.controller.poll()
        self.node.capture_current_pose.assert_not_called()
        self.node.zero_velocity_hold_ok.return_value = True
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.SUCCEEDED)

    def test_two_cm_policy_rejects_drift_beyond_two_cm(self):
        target = NavigationTarget('point', 'point', 1., 0., 0., POSITION_TOLERANCE_M, YAW_TOLERANCE_DEG)
        self.assertTrue(self.controller.start([target], 'two-cm-drift'))
        self.controller.poll(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.capture_current_pose.return_value = {'x':1.021, 'y':0., 'yaw_rad':0.}
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertFalse(self.controller.arrivals[-1]['accepted'])

    def test_twenty_deg_policy_rejects_heading_even_with_exact_position(self):
        target = NavigationTarget('point', 'point', 1., 0., 0., POSITION_TOLERANCE_M, YAW_TOLERANCE_DEG)
        self.assertTrue(self.controller.start([target], 'twenty-deg-overrun'))
        self.controller.poll(); self.accept()
        self.result.set_result(NS(status=4)); self.controller.poll()
        self.node.capture_current_pose.return_value = {'x':1., 'y':0., 'yaw_rad':math.radians(-10.1)}
        self.controller.poll()
        self.assertEqual(self.controller.state, self.controller.FAILED)
        self.assertEqual(self.controller.index, 0)
        self.assertFalse(self.controller.arrivals[-1]['accepted'])


if __name__ == '__main__':
    unittest.main()
