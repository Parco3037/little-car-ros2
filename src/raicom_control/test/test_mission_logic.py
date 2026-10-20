import unittest

from raicom_control.mission_logic import (
    requires_joint_state,
    should_run_arrival_action,
    validate_mission_configuration,
)


class MissionLogicTest(unittest.TestCase):
    def test_navigation_only_does_not_require_gripper(self) -> None:
        error = validate_mission_configuration(
            goal_configured=True,
            arrival_action_enabled=False,
            gripper_calibrated=False,
            gripper_open_pulse=0.0,
            gripper_close_pulse=0.0,
            gripper_max_delta_pulse=80.0,
        )
        self.assertIsNone(error)
        self.assertFalse(requires_joint_state(False))
        self.assertFalse(should_run_arrival_action(False))

    def test_missing_goal_always_blocks(self) -> None:
        error = validate_mission_configuration(
            goal_configured=False,
            arrival_action_enabled=False,
            gripper_calibrated=False,
            gripper_open_pulse=0.0,
            gripper_close_pulse=0.0,
            gripper_max_delta_pulse=80.0,
        )
        self.assertEqual(error, 'home_pick_station is not configured')

    def test_enabled_arrival_action_requires_safe_gripper_calibration(self) -> None:
        error = validate_mission_configuration(
            goal_configured=True,
            arrival_action_enabled=True,
            gripper_calibrated=False,
            gripper_open_pulse=0.0,
            gripper_close_pulse=0.0,
            gripper_max_delta_pulse=80.0,
        )
        self.assertEqual(error, 'gripper is not calibrated')
        self.assertTrue(requires_joint_state(True))
        self.assertTrue(should_run_arrival_action(True))

    def test_enabled_arrival_action_rejects_unsafe_pulse_delta(self) -> None:
        error = validate_mission_configuration(
            goal_configured=True,
            arrival_action_enabled=True,
            gripper_calibrated=True,
            gripper_open_pulse=300.0,
            gripper_close_pulse=500.0,
            gripper_max_delta_pulse=80.0,
        )
        self.assertEqual(
            error,
            'gripper pulse delta is zero or exceeds the configured safety limit',
        )


if __name__ == '__main__':
    unittest.main()
