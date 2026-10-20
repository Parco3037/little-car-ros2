import math
import unittest

from raicom_control.mapping_logic import (
    directional_clearance,
    distance_2d,
    normalize_angle,
    yaw_from_quaternion,
)


class MappingLogicTest(unittest.TestCase):
    def test_angle_wrap_and_yaw(self) -> None:
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(yaw_from_quaternion(math.sin(math.pi / 4), math.cos(math.pi / 4)), math.pi / 2)

    def test_distance(self) -> None:
        self.assertAlmostEqual(distance_2d(1.0, 2.0, 4.0, 6.0), 5.0)

    def test_directional_clearance_filters_invalid_and_outside_sector(self) -> None:
        ranges = [1.0, math.nan, 0.4, 2.0, math.inf]
        clearance, count = directional_clearance(
            ranges,
            angle_min=-0.2,
            angle_increment=0.1,
            direction=0.0,
            half_width=0.11,
            range_min=0.05,
            range_max=10.0,
        )
        self.assertEqual(count, 2)
        self.assertAlmostEqual(clearance, 0.4)


if __name__ == '__main__':
    unittest.main()
