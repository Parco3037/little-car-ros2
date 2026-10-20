from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'deploy'))
from runtime_fusion_gate import RuntimeFusionGate, source_sample_is_fresh
from scan_keyframe_experiment import TrackingConfig


class RuntimeGateTest(unittest.TestCase):
    def test_tracker_gap_matches_documented_runtime_recovery_budget(self):
        self.assertEqual(TrackingConfig().max_gap_sec,
                         RuntimeFusionGate.MAX_RECOVERY_SEC)

    def test_source_age_classifier_never_accepts_future_or_nonfinite_time(self):
        self.assertTrue(source_sample_is_fresh(.3, .3))
        self.assertFalse(source_sample_is_fresh(.3146746, .3))
        for age in (-.001, float('nan'), float('inf'), None):
            with self.assertRaises(ValueError):
                source_sample_is_fresh(age, .3)

    def ready(self):
        g = RuntimeFusionGate()
        for s in g.LIMITS: g.observe(s, 10.)
        self.assertTrue(g.evaluate(10., 'TRACKING')['valid'])
        return g

    def pause(self, g):
        for s in ('laser','imu'): g.observe(s, 10.31)
        state = g.evaluate(10.31, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED'); self.assertFalse(state['valid'])

    def test_short_ekf_delay_requires_three_ekf_two_laser_then_same_state(self):
        g = self.ready(); self.pause(g)
        for t in (10.35, 10.40):
            for s in g.LIMITS: g.observe(s, t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])
        g.observe('ekf', 10.45); g.observe('imu', 10.45)
        self.assertTrue(g.evaluate(10.45, 'TRACKING')['valid'])
        self.assertEqual((g.pauses,g.resumptions),(1,1))
        self.assertTrue(g.established); self.assertIsNone(g.failure)

    def test_fresh_ekf_alone_does_not_resume(self):
        g = self.ready(); self.pause(g)
        for t in (10.35,10.4,10.45,10.5):
            g.observe('ekf', t); g.observe('imu', t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])

    def test_observed_314ms_imu_jitter_pauses_then_same_origin_resumes(self):
        g = self.ready()
        # Exact shape of the 2026-09-01 pickup failure: laser and EKF were
        # fresh while one IMU delivery interval reached 314.7ms.
        g.observe('laser', 10.1990566)
        g.observe('ekf', 10.0971699)
        state = g.evaluate(10.3146746, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED')
        self.assertFalse(state['valid'])
        self.assertEqual(state['pause_sources'], ['imu'])
        self.assertIsNone(g.failure)
        for t in (10.34, 10.37):
            for source in g.LIMITS:
                g.observe(source, t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])
        for source in ('imu', 'ekf'):
            g.observe(source, 10.40)
        self.assertTrue(g.evaluate(10.40, 'TRACKING')['valid'])
        self.assertEqual((g.pauses, g.resumptions), (1, 1))

    def test_imu_pause_never_reopens_from_imu_alone(self):
        g = self.ready()
        g.observe('laser', 10.2); g.observe('ekf', 10.2)
        self.assertEqual(g.evaluate(10.31, 'TRACKING')['state'], 'PAUSED')
        for t in (10.34, 10.37, 10.40, 10.43):
            g.observe('imu', t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])

    def test_imu_stall_over_one_second_latches(self):
        g = self.ready()
        g.observe('laser', 10.2); g.observe('ekf', 10.2)
        self.assertEqual(g.evaluate(10.31, 'TRACKING')['state'], 'PAUSED')
        g.observe('laser', 11.01); g.observe('ekf', 11.01)
        self.assertEqual(g.evaluate(11.01, 'TRACKING')['state'], 'LOST')
        self.assertIn('independent sensor lost', g.failure)

    def test_long_ekf_stall_latches(self):
        g = self.ready(); self.pause(g)
        g.observe('laser',11.01); g.observe('imu',11.01)
        self.assertEqual(g.evaluate(11.01,'TRACKING')['state'],'LOST')
        for s in g.LIMITS: g.observe(s,11.02)
        self.assertFalse(g.evaluate(11.02,'TRACKING')['valid'])

    def test_laser_loss_not_masked(self):
        g = self.ready(); self.pause(g)
        g.observe('imu',10.9); g.observe('ekf',10.9)
        self.assertEqual(g.evaluate(10.9,'TRACKING')['state'],'LOST')

    def test_clock_reversal_and_tracker_reset_not_masked(self):
        for mode in ('clock','tracker'):
            g = self.ready(); self.pause(g)
            if mode == 'clock': g.observe('ekf',9.99)
            state = g.evaluate(10.32, 'TRACKING' if mode == 'clock' else 'UNINITIALIZED')
            self.assertEqual(state['state'],'LOST')

    def test_live_scan_processing_pause_never_accepts_old_measurement(self):
        g = self.ready()
        g.observe_scan_input(10.4)
        for s in ('imu', 'ekf'): g.observe(s, 10.48)
        state = g.evaluate(10.48, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED')
        self.assertFalse(state['valid'])
        self.assertEqual(state['source_stamps']['laser'], 10.)
        for t in (10.50, 10.55):
            g.observe_scan_input(t)
            for s in g.LIMITS: g.observe(s, t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])
        for s in ('imu','ekf'): g.observe(s, 10.58)
        self.assertTrue(g.evaluate(10.58, 'TRACKING')['valid'])
        self.assertEqual((g.pauses, g.resumptions), (1,1))

    def test_received_scans_and_fresh_predictions_never_replace_matching(self):
        g = self.ready()
        for t in (10.48,10.52,10.56,10.59):
            g.observe_scan_input(t)
            for s in ('imu','ekf'): g.observe(s,t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])
        g.observe_scan_input(10.61)
        for s in ('imu','ekf'): g.observe(s,10.61)
        self.assertEqual(g.evaluate(10.61,'TRACKING')['state'],'PAUSED')
        g.observe_scan_input(11.01)
        for s in ('imu','ekf'): g.observe(s,11.01)
        self.assertEqual(g.evaluate(11.01,'TRACKING')['state'],'LOST')

    def test_no_pause_without_any_recent_scan_receipt(self):
        g = self.ready()
        for s in ('imu','ekf'): g.observe(s,10.48)
        self.assertEqual(g.evaluate(10.48,'TRACKING')['state'],'LOST')

    def test_observed_direct_scan_receipt_gap_pauses_then_recovers(self):
        g = self.ready()
        # Exact shape of the 2026-09-02 failure: the accepted laser age was
        # 479.6 ms and the raw-scan receipt age was 382.6 ms.  Both are inside
        # the existing one-second recovery budget, so output must close while
        # the same tracker waits for independent fresh confirmations.
        g.observe('laser', 10.1252646)
        g.observe_scan_input(10.2222744)
        g.observe('imu', 10.5409477)
        g.observe('ekf', 10.5194404)
        state = g.evaluate(10.6048815, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED')
        self.assertEqual(state['pause_sources'], ['scan_transport'])
        self.assertIsNone(g.failure)

        for t in (10.70, 10.75):
            g.observe_scan_input(t)
            for source in g.LIMITS:
                g.observe(source, t)
            self.assertFalse(g.evaluate(t, 'TRACKING')['valid'])
        g.observe_scan_input(10.80)
        for source in ('imu', 'ekf'):
            g.observe(source, 10.80)
        self.assertTrue(g.evaluate(10.80, 'TRACKING')['valid'])
        self.assertEqual((g.pauses, g.resumptions), (1, 1))

    def test_direct_scan_receipt_gap_still_latches_at_one_second(self):
        g = self.ready()
        g.observe_scan_input(10.)
        for source in ('imu', 'ekf'):
            g.observe(source, 10.48)
        state = g.evaluate(10.48, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED')
        self.assertEqual(state['pause_sources'], ['scan_transport'])
        for source in ('imu', 'ekf'):
            g.observe(source, 11.01)
        self.assertEqual(g.evaluate(11.01, 'TRACKING')['state'], 'LOST')

    def test_existing_pause_survives_bounded_scan_transport_delay(self):
        g = self.ready()
        g.observe_scan_input(10.0)
        g.observe('laser', 10.2)
        state = g.evaluate(10.31, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED')
        self.assertEqual(state['pause_sources'], ['imu', 'ekf'])

        for source in ('imu', 'ekf'):
            g.observe(source, 10.65)
        state = g.evaluate(10.66, 'TRACKING')
        self.assertEqual(state['state'], 'PAUSED')
        self.assertIn('scan_transport', state['pause_sources'])
        self.assertIsNone(g.failure)

        g.observe_scan_input(10.70)
        for source in g.LIMITS:
            g.observe(source, 10.70)
        self.assertFalse(g.evaluate(10.70, 'TRACKING')['valid'])
        g.observe_scan_input(10.75)
        for source in g.LIMITS:
            g.observe(source, 10.75)
        self.assertTrue(g.evaluate(10.75, 'TRACKING')['valid'])
        self.assertEqual((g.pauses, g.resumptions), (1, 1))

    def test_existing_pause_still_latches_after_one_second_transport_loss(self):
        g = self.ready()
        g.observe_scan_input(10.0)
        g.observe('laser', 10.2)
        self.assertEqual(g.evaluate(10.31, 'TRACKING')['state'], 'PAUSED')
        for source in ('imu', 'ekf'):
            g.observe(source, 11.01)
        self.assertEqual(g.evaluate(11.01, 'TRACKING')['state'], 'LOST')
        self.assertIn('independent sensor lost', g.failure)

    def test_received_clock_faults_latch(self):
        for bad_stamp in (9.99, float('nan'), float('inf'), 11.):
            g = self.ready(); g.observe_scan_input(10.)
            g.observe_scan_input(bad_stamp)
            self.assertEqual(g.evaluate(10.1,'TRACKING')['state'],'LOST')

    def test_processing_pause_bounds_imu_delay_but_rejects_tracker_reset(self):
        g = self.ready(); g.observe_scan_input(10.48); g.observe('ekf',10.48)
        state = g.evaluate(10.48, 'TRACKING')
        self.assertEqual(state['state'],'PAUSED')
        self.assertIn('imu', state['pause_sources'])

        g = self.ready(); g.observe_scan_input(10.48)
        for source in ('imu','ekf'):
            g.observe(source,10.48)
        state = g.evaluate(10.48, 'UNINITIALIZED')
        self.assertEqual(state['state'],'LOST')
