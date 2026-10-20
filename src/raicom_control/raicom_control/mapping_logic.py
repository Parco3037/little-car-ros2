"""Pure helpers for the bounded home mapping motion."""

import math
from typing import Iterable


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(z: float, w: float) -> float:
    """Return planar yaw for a quaternion whose x/y components are zero."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def distance_2d(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def directional_clearance(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    direction: float,
    half_width: float,
    range_min: float,
    range_max: float,
) -> tuple[float | None, int]:
    """Return the closest valid range and sample count in an angular sector."""
    valid = []
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if abs(normalize_angle(angle - direction)) > half_width:
            continue
        if math.isfinite(value) and range_min <= value <= range_max:
            valid.append(float(value))
    return (min(valid), len(valid)) if valid else (None, 0)
