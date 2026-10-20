import numpy as np
import pytest

from rgbd_grasp.grasp_core import (
    Rejected,
    bounded_search_positions,
    interpolate,
    side_grasp_waypoints,
    stable_target,
    validate_arm_pulses,
    validate_joint_step,
)


def make_samples(point=(0.20, 0.01, 0.20), count=5, stamp=10.0):
    return [
        {
            'stamp_s': stamp - (count - index - 1) * 0.01,
            'frame': 'base_link',
            'point': point,
        }
        for index in range(count)
    ]


def test_stable_target_accepts_fresh_samples():
    result = stable_target(
        make_samples(),
        10.1,
        'base_link',
        5,
        0.5,
        0.005,
        [0.08, 0.35, -0.20, 0.20, 0.08, 0.30],
    )
    assert np.allclose(result, [0.20, 0.01, 0.20])


@pytest.mark.parametrize('change', ['stale', 'spread', 'workspace'])
def test_stable_target_rejects_invalid_data(change):
    samples = make_samples()
    now = 10.1
    workspace = [0.08, 0.35, -0.20, 0.20, 0.08, 0.30]
    if change == 'stale':
        now = 11.0
    elif change == 'spread':
        samples[-1]['point'] = (0.24, 0.01, 0.20)
    else:
        samples = make_samples(point=(0.50, 0.01, 0.20))
    with pytest.raises(Rejected):
        stable_target(
            samples, now, 'base_link', 5, 0.5, 0.005, workspace)


def test_interpolation_never_exceeds_five_millimetres():
    points = interpolate([0.0, 0.0, 0.0], [0.011, 0.0, 0.0], 0.005)
    previous = np.zeros(3)
    for point in points:
        assert np.linalg.norm(point - previous) <= 0.005 + 1e-12
        previous = point
    assert np.allclose(points[-1], [0.011, 0.0, 0.0])


def test_side_grasp_lifts_five_centimetres():
    pregrasp, approach, lift = side_grasp_waypoints(
        [0.20, 0.0, 0.20],
        [0.0, 0.0, -0.025],
        0.05,
        0.05,
        0.005,
    )
    assert np.allclose(pregrasp, [0.15, 0.0, 0.175])
    assert np.allclose(approach[-1], [0.20, 0.0, 0.175])
    assert np.allclose(lift[-1], [0.20, 0.0, 0.225])


def test_servo_validation_enforces_id1_and_fixed_id5():
    limits = [
        [350, 550],
        [125, 875],
        [0, 1000],
        [42, 958],
        [0, 1000],
    ]
    result = validate_arm_pulses(
        [450, 600, 400, 300, 500],
        limits,
        [350, 550],
        875,
    )
    assert result.tolist() == [450, 600, 400, 300, 875]
    with pytest.raises(Rejected, match='ID1'):
        validate_arm_pulses(
            [300, 600, 400, 300, 875],
            limits,
            [350, 550],
            875,
        )


def test_joint_jump_is_bounded():
    validate_joint_step([450, 500, 500, 500, 875],
                        [455, 510, 490, 500, 875], 20)
    with pytest.raises(Rejected):
        validate_joint_step([450, 500, 500, 500, 875],
                            [550, 500, 500, 500, 875], 20)


def test_search_positions_alternate_without_drift():
    assert bounded_search_positions(
        450, [350, 550], 20, 2) == [430, 470, 410, 490]


def test_search_positions_clip_without_duplicates():
    assert bounded_search_positions(
        355, [350, 550], 20, 2) == [350, 375, 395]
    with pytest.raises(Rejected):
        bounded_search_positions(300, [350, 550], 20, 2)
