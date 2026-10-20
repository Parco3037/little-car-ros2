import numpy as np

from raicom_control.aurora_observer_logic import (
    Roi,
    StabilityGate,
    observe_roi,
    robust_depth_mm,
)


def test_robust_depth_rejects_invalid_patch():
    depth = np.zeros((20, 20), dtype=np.uint16)
    value, ratio, reason = robust_depth_mm(depth, Roi(2, 2, 10, 10))
    assert value is None
    assert ratio == 0.0
    assert 'insufficient' in reason


def test_observation_projects_camera_point_and_gate_needs_three_frames():
    depth = np.full((40, 60), 1000, dtype=np.uint16)
    roi = Roi(25, 15, 35, 25)
    obs = observe_roi(depth, roi, fx=100.0, fy=100.0, cx=30.0, cy=20.0)
    assert obs.ready
    assert abs(obs.x_m) < 0.01
    assert abs(obs.y_m) < 0.01
    assert abs(obs.z_m - 1.0) < 1e-6

    gate = StabilityGate(required_frames=3, max_pixel_delta=3, max_xyz_delta_m=0.02)
    assert gate.push(obs) is False
    assert gate.push(obs) is False
    assert gate.push(obs) is True


def test_gate_resets_on_large_motion_or_invalid_observation():
    depth = np.full((40, 60), 1000, dtype=np.uint16)
    first = observe_roi(depth, Roi(25, 15, 35, 25), fx=100, fy=100, cx=30, cy=20)
    moved = observe_roi(depth, Roi(35, 15, 45, 25), fx=100, fy=100, cx=30, cy=20)
    gate = StabilityGate(required_frames=3, max_pixel_delta=3, max_xyz_delta_m=0.02)
    gate.push(first)
    assert gate.push(moved) is False
    assert gate.count == 1
    invalid = observe_roi(np.zeros_like(depth), Roi(25, 15, 35, 25), fx=100, fy=100, cx=30, cy=20)
    assert gate.push(invalid) is False
    assert gate.count == 0
