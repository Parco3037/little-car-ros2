"""Pure validation helpers for the fixed-scene pick/place mission."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any


ARM_STAGES = ('pregrasp', 'descend', 'grasp', 'lift', 'stow', 'release', 'final_stow')
ALLOWED_SERVO_IDS = {1, 2, 3, 4, 5, 10}


@dataclass(frozen=True)
class SceneStartConfig:
    goal_configured: bool
    place_configured: bool
    arm_calibrated: bool
    gripper_calibrated: bool
    arm_profile_json: str
    stable_wait_ms: int = 500
    target_timeout_sec: float = 3.0


def parse_arm_profile(profile_json: str) -> dict[str, tuple[tuple[int, float], ...]]:
    """Parse a complete, bounded pulse profile for the fixed scene."""

    raw: Any = json.loads(profile_json)
    if not isinstance(raw, dict):
        raise ValueError('arm_profile_json must be an object')
    missing = [stage for stage in ARM_STAGES if stage not in raw]
    if missing:
        raise ValueError('arm profile missing stages: ' + ','.join(missing))
    parsed: dict[str, tuple[tuple[int, float], ...]] = {}
    for stage in ARM_STAGES:
        commands = raw[stage]
        if not isinstance(commands, list) or not commands:
            raise ValueError(f'arm profile stage {stage} is empty')
        values: list[tuple[int, float]] = []
        for command in commands:
            if not isinstance(command, (list, tuple)) or len(command) != 2:
                raise ValueError(f'arm profile stage {stage} has an invalid command')
            servo_id = int(command[0])
            pulse = float(command[1])
            if servo_id not in ALLOWED_SERVO_IDS:
                raise ValueError(f'arm profile stage {stage} has an invalid servo id')
            if not 0.0 <= pulse <= 1000.0:
                raise ValueError(f'arm profile stage {stage} has an out-of-range pulse')
            values.append((servo_id, pulse))
        parsed[stage] = tuple(values)
    return parsed


def validate_scene_start(config: SceneStartConfig) -> str | None:
    if not config.goal_configured:
        return 'pickup observation pose is not configured'
    if not config.place_configured:
        return 'left red-region place pose is not configured'
    if not config.arm_calibrated:
        return 'arm calibration is not approved for the +4 mm arm'
    if not config.gripper_calibrated:
        return 'two-finger gripper calibration is not approved'
    if not 100 <= config.stable_wait_ms <= 2000:
        return 'stable_wait_ms must be within 100..2000'
    if not 0.5 <= config.target_timeout_sec <= 30.0:
        return 'target_timeout_sec must be within 0.5..30'
    try:
        parse_arm_profile(config.arm_profile_json)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        return str(exc)
    return None


def target_is_usable(*, ready: bool, age_sec: float, max_age_sec: float, x_m: float, y_m: float, z_m: float) -> bool:
    """Conservative final gate before a pregrasp command."""

    if not ready or not 0.0 <= age_sec <= max_age_sec:
        return False
    return -0.60 <= x_m <= 0.60 and -0.45 <= y_m <= 0.45 and 0.15 <= z_m <= 2.50
