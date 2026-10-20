import math
import unittest

from raicom_control.odom_bridge_logic import (
    pose_sample_is_valid,
    validate_bridge_configuration,
)


class OdomBridgeLogicTest(unittest.TestCase):
    def test_valid_pose_sample(self) -> None:
        self.assertTrue(pose_sample_is_valid((1.0, 2.0, 0.0, 0.0, 0.0, 0.5, 0.866)))

    def test_invalid_pose_sample_is_rejected(self) -> None:
        self.assertFalse(pose_sample_is_valid((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)))
        self.assertFalse(pose_sample_is_valid((0.0, 0.0, math.nan, 0.0, 0.0, 0.0, 1.0)))
        self.assertFalse(pose_sample_is_valid((0.0, 0.0)))

    def test_feedback_and_same_frame_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            validate_bridge_configuration('/odom', '/odom', 'odom', 'base_footprint')
        with self.assertRaises(ValueError):
            validate_bridge_configuration('/odom_raw', '/odom', 'odom', 'odom')


if __name__ == '__main__':
    unittest.main()
