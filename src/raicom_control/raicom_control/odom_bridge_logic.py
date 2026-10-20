"""Pure validation helpers for the RAICOM navigation odometry bridge."""

import math
from collections.abc import Iterable


def validate_bridge_configuration(
    input_topic: str,
    output_topic: str,
    parent_frame: str,
    child_frame: str,
) -> None:
    """Reject configurations that could form a feedback loop or invalid TF edge."""
    if not input_topic.strip() or not output_topic.strip():
        raise ValueError('odometry input and output topics must be non-empty')
    if input_topic == output_topic:
        raise ValueError('odometry input and output topics must differ')
    if not parent_frame.strip() or not child_frame.strip():
        raise ValueError('odometry parent and child frames must be non-empty')
    if parent_frame == child_frame:
        raise ValueError('odometry parent and child frames must differ')


def pose_sample_is_valid(values: Iterable[float]) -> bool:
    """Return true for a finite XYZ + quaternion pose with a usable quaternion."""
    sample = tuple(values)
    if len(sample) != 7 or not all(math.isfinite(value) for value in sample):
        return False
    quaternion_norm_squared = sum(value * value for value in sample[3:])
    return quaternion_norm_squared > 1.0e-12
