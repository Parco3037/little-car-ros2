"""Pure helpers for recording fixed-scene map poses.

The recorder deliberately edits only the ``scene_mission`` parameter block.
It never invents a pose and never changes the arm or base command paths.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass


@dataclass(frozen=True)
class ScenePose:
    x: float
    y: float
    yaw: float


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-9:
        raise ValueError("quaternion has zero length")
    _, _, z_n, w_n = (value / norm for value in values)
    yaw = math.atan2(2.0 * w_n * z_n, 1.0 - 2.0 * z_n * z_n)
    if yaw >= math.pi:
        yaw -= 2.0 * math.pi
    return yaw


def validate_pose(pose: ScenePose) -> None:
    if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
        raise ValueError("scene pose contains a non-finite value")
    if not -20.0 <= pose.x <= 20.0 or not -20.0 <= pose.y <= 20.0:
        raise ValueError("scene pose is outside the 20 m home-map guard")
    if not -math.pi - 1.0e-6 <= pose.yaw <= math.pi + 1.0e-6:
        raise ValueError("scene yaw must be within -pi..pi")


def update_scene_yaml(text: str, station: str, pose: ScenePose) -> str:
    """Return YAML with one recorded station updated atomically by the caller."""

    validate_pose(pose)
    if station not in {"pick", "place"}:
        raise ValueError("station must be pick or place")
    marker = "scene_mission:"
    if marker not in text:
        raise ValueError("scene_mission parameter block is missing")

    prefix = "goal" if station == "pick" else "place"
    replacements = {
        f"{prefix}_configured": "true",
        f"{prefix}_x": f"{pose.x:.6f}",
        f"{prefix}_y": f"{pose.y:.6f}",
        f"{prefix}_yaw": f"{pose.yaw:.6f}",
    }
    # Restrict substitutions to the first, four-space-indented scene block;
    # the file also contains a separate home_mission block.
    marker_end = text.index(marker) + len(marker)
    next_block = re.search(r"(?m)^\S[^\n]*:", text[marker_end:])
    block_end = marker_end + next_block.start() if next_block else len(text)
    head, block, tail = text[:marker_end], text[marker_end:block_end], text[block_end:]
    for key, value in replacements.items():
        pattern = re.compile(rf"(?m)^(\s{{4}}{re.escape(key)}:\s*).*$")
        block, count = pattern.subn(rf"\g<1>{value}", block, count=1)
        if count != 1:
            raise ValueError(f"{key} is missing from scene_mission block")
    return head + block + tail
