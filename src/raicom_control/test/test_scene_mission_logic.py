import json

import pytest

from raicom_control.scene_mission_logic import (
    ARM_STAGES,
    SceneStartConfig,
    parse_arm_profile,
    target_is_usable,
    validate_scene_start,
)


def _profile() -> str:
    return json.dumps({stage: [[1, 500.0]] for stage in ARM_STAGES})


def _config(**overrides) -> SceneStartConfig:
    values = dict(
        goal_configured=True,
        place_configured=True,
        arm_calibrated=True,
        gripper_calibrated=True,
        arm_profile_json=_profile(),
    )
    values.update(overrides)
    return SceneStartConfig(**values)


def test_default_scene_configuration_is_fail_closed():
    error = validate_scene_start(
        SceneStartConfig(False, False, False, False, '')
    )
    assert error == 'pickup observation pose is not configured'


def test_complete_profile_requires_every_stage_and_bounds_pulses():
    parsed = parse_arm_profile(_profile())
    assert tuple(parsed) == ARM_STAGES
    bad = json.loads(_profile())
    bad.pop('descend')
    with pytest.raises(ValueError, match='missing stages'):
        parse_arm_profile(json.dumps(bad))

    bad = json.loads(_profile())
    bad['grasp'] = [[1, 1001]]
    with pytest.raises(ValueError, match='out-of-range'):
        parse_arm_profile(json.dumps(bad))


def test_target_gate_rejects_stale_or_out_of_workspace_points():
    assert target_is_usable(
        ready=True, age_sec=0.1, max_age_sec=0.5,
        x_m=0.0, y_m=0.0, z_m=0.5,
    )
    assert not target_is_usable(
        ready=False, age_sec=0.1, max_age_sec=0.5,
        x_m=0.0, y_m=0.0, z_m=0.5,
    )
    assert not target_is_usable(
        ready=True, age_sec=0.6, max_age_sec=0.5,
        x_m=0.0, y_m=0.0, z_m=0.5,
    )
    assert not target_is_usable(
        ready=True, age_sec=0.1, max_age_sec=0.5,
        x_m=0.7, y_m=0.0, z_m=0.5,
    )
