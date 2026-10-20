import math
import unittest

from raicom_control.guard_logic import (
    AccelerationLimiter, MotionEnableLease, ZERO_COMMAND, VelocityCommand,
    WatchdogState,
    sanitize_command,
)


class GuardLogicTest(unittest.TestCase):
    def test_watchdog_starts_stopped_and_times_out(self) -> None:
        state = WatchdogState(0.25)
        self.assertEqual(state.command_at(100.0), ZERO_COMMAND)
        moving = VelocityCommand(0.1, -0.1, 0.35)
        state.update(moving, 100.0)
        self.assertEqual(state.command_at(100.249), moving)
        self.assertEqual(state.command_at(100.251), ZERO_COMMAND)

    def test_watchdog_fails_closed_on_clock_regression(self) -> None:
        state = WatchdogState(0.25)
        state.update(VelocityCommand(0.1, 0.0, 0.0), 10.0)
        self.assertEqual(state.command_at(9.9), ZERO_COMMAND)

    def test_sanitize_clamps_and_rejects_non_finite_values(self) -> None:
        command = sanitize_command((1.0, -1.0, math.nan), 0.10, 0.35)
        self.assertEqual(command, VelocityCommand(0.10, -0.10, 0.0))

    def test_timeout_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            WatchdogState(0.0)

    def test_acceleration_limiter_ramps_and_resets(self) -> None:
        limiter = AccelerationLimiter(0.5, 1.0)
        requested = VelocityCommand(0.25, -0.25, 0.35)
        self.assertEqual(limiter.apply(requested, 10.0), ZERO_COMMAND)
        limited = limiter.apply(requested, 10.1)
        self.assertAlmostEqual(limited.linear_x, 0.05)
        self.assertAlmostEqual(limited.linear_y, -0.05)
        self.assertAlmostEqual(limited.angular_z, 0.1)
        limiter.reset(10.2)
        self.assertEqual(limiter.apply(requested, 10.2), ZERO_COMMAND)


class MotionEnableLeaseTest(unittest.TestCase):
    def test_requires_the_small_four_field_message(self) -> None:
        lease = MotionEnableLease()
        message = {
            'boot_id': 'boot-a', 'mission_id': 'mission-a',
            'enabled': True, 'expires_at': 10.5,
        }
        self.assertTrue(lease.receive(message, 10.0))
        self.assertTrue(lease.valid_at(10.5))
        self.assertFalse(lease.valid_at(10.501))
        self.assertFalse(lease.receive(dict(message, extra=True), 10.1))

    def test_disabled_or_malformed_message_keeps_the_guard_stopped(self) -> None:
        lease = MotionEnableLease()
        self.assertFalse(lease.valid_at(1.0))
        self.assertTrue(lease.receive({
            'boot_id': 'boot-a', 'mission_id': 'mission-a',
            'enabled': False, 'expires_at': 20.0,
        }, 1.0))
        self.assertFalse(lease.valid_at(1.0))
        self.assertFalse(lease.receive({
            'boot_id': 'boot-a', 'mission_id': 'mission-a',
            'enabled': True, 'expires_at': '20.0',
        }, 1.0))


if __name__ == '__main__':
    unittest.main()
