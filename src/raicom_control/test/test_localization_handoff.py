import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'deploy'))
from odom_continuity import ContinuityAnchor
from raicom_control.guard_logic import LocalizationLease


class LeaseTest(unittest.TestCase):
    def setUp(self):
        self.lease = LocalizationLease('test-session')
        self.message = dict(session='test-session', sequence=1, state='ACTIVE', valid=True,
                            source_stamps=dict(laser=10., imu=10., ekf=10.))

    def test_initially_stopped(self):
        self.assertFalse(self.lease.valid_at(20, 10))

    def test_heartbeat_expires_even_with_frozen_ros_time(self):
        self.assertTrue(self.lease.receive(self.message, 20, 10))
        self.assertTrue(self.lease.valid_at(20.1, 10.1))
        self.assertFalse(self.lease.valid_at(20.301, 10.1))

    def test_fresh_heartbeat_cannot_renew_stale_sources(self):
        self.assertTrue(self.lease.receive(self.message, 20, 10))
        self.message['sequence'] += 1
        self.assertFalse(self.lease.receive(self.message, 20.35, 10.35))

    def test_foreign_replayed_invalid_messages_fail_closed(self):
        self.assertTrue(self.lease.receive(self.message, 20, 10))
        for key, value in [('sequence', 1), ('sequence', True), ('session', 'other'),
                           ('valid', 1), ('valid', False), ('state', 'STANDBY')]:
            message = copy.deepcopy(self.message)
            message['sequence'] = 2
            message[key] = value
            self.assertFalse(self.lease.receive(message, 20.1, 10.1))
            self.assertFalse(self.lease.valid_at(20.1, 10.1))
        for message in [None, [], {}, 'x']:
            self.assertFalse(self.lease.receive(message, 20.1, 10.1))

    def test_source_and_clock_errors(self):
        for value in [None, True, math.nan, math.inf, 11., 9.]:
            message = copy.deepcopy(self.message)
            message['source_stamps']['laser'] = value
            self.assertFalse(LocalizationLease('test-session').receive(message, 20, 10))
        self.assertTrue(self.lease.receive(self.message, 20, 10))
        self.assertFalse(self.lease.valid_at(19.9, 10))

    def test_ros_regression_and_source_regression(self):
        self.assertTrue(self.lease.receive(self.message, 20, 10))
        self.assertFalse(self.lease.valid_at(20.1, 9.9))
        self.message['sequence'] = 2
        self.message['source_stamps']['laser'] = 9.99
        self.assertFalse(self.lease.receive(self.message, 20.1, 10.1))


class AnchorTest(unittest.TestCase):
    def test_first_pose_exactly_preserved(self):
        old, new = [2., -1., 3.1], [.03, -.04, -.2]
        anchor = ContinuityAnchor(old, new)
        np.testing.assert_allclose(anchor.transform(new), old, atol=1e-12)

    def test_motion_rotates_not_reanchors(self):
        anchor = ContinuityAnchor([2, -1, math.pi/2], [0, 0, 0])
        np.testing.assert_allclose(anchor.transform([.1, 0, .2]),
                                   [2, -.9, math.pi/2+.2], atol=1e-12)

    def test_covariance_axes_and_psd(self):
        anchor = ContinuityAnchor([0, 0, math.pi/2], [0, 0, 0])
        c = np.diag([1, 2, 3, 4, 5, 6])
        actual = np.array(anchor.covariance(c.ravel())).reshape(6, 6)
        np.testing.assert_allclose(np.diag(actual), [2, 1, 3, 5, 4, 6])
        self.assertTrue(np.linalg.eigvalsh(actual).min() >= 0)

    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):
            ContinuityAnchor([math.nan, 0, 0], [0, 0, 0])
