import json
import math
from pathlib import Path

import pytest
import yaml

from raicom_control.nav_scan_geometry import (
    NAV_SCAN_TOPIC, footprint_points, mask_body_returns, navigation_scan_profile, inside_body_mask)

ROOT = Path(__file__).parents[1]
BODY = footprint_points(json.loads((ROOT/'config/fixed_route_footprint.json').read_text())['points_xy_m'])


def test_captured_body_return_removed_without_erasing_same_bearing_external_object():
    # Fresh Pi TF: lidar is 73.021 mm forward of base_footprint. Index 341
    # lies at the edge of the existing hardware angular mask.
    output, removed = mask_body_returns([.08, .141], 341*2*math.pi/500, 1e-8,
        .05, 2., (.07302117405202284, 0., .0927), (0., 0., 0., 1.), BODY)
    assert removed == [0]
    assert math.isnan(output[0]) and output[1] == .141


def test_thin_external_single_point_and_invalid_beams_preserved():
    original = [float('nan'), .110, float('inf'), 3.]
    output, removed = mask_body_returns(original, math.pi, .001, .05, 2.,
        (0., 0., 0.), (0., 0., 0., 1.), BODY)
    assert removed == [] and output[1:] == original[1:]
    assert math.isnan(output[0])


def test_full_quaternion_transform_and_boundary_retention():
    square = footprint_points([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
    output, removed = mask_body_returns([.1,.05], 0., .01, .01, 2.,
        (0.,0.,0.), (0.,0.,0.,1.), square)
    assert math.isnan(output[0]) and removed == [0, 1]
    _, removed = mask_body_returns([.15], 0., .01, .01, 2.,
        (.1,0.,0.), (0.,0.,1.,0.), square)
    assert removed == [0]


def test_explicit_five_millimetre_margin_with_exact_rounded_corners():
    square = footprint_points([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
    assert inside_body_mask(.105, 0., square)
    assert not inside_body_mask(.105001, 0., square)
    assert inside_body_mask(.103, .103, square)
    assert not inside_body_mask(.104, .104, square)


@pytest.mark.parametrize('angles,quat', [((0.,0.),(0.,0.,0.,1.)), ((0.,.01),(0.,0.,0.,0.)), ((float('nan'),.01),(0.,0.,0.,1.))])
def test_invalid_transform_or_geometry_rejected(angles,quat):
    with pytest.raises(ValueError):
        mask_body_returns([.1], *angles, .05, 2., (0.,0.,0.), quat, BODY)


@pytest.mark.parametrize('filename', ['nav2_params_workspace.yaml','nav2_params_home_straight_low_load.yaml'])
def test_only_navigation_costmap_inputs_change_and_source_remains_intact(tmp_path,filename):
    source = ROOT/'config'/filename
    original = source.read_bytes()
    path, footprint = navigation_scan_profile(source, tmp_path)
    generated = yaml.safe_load(path.read_text())
    expected = yaml.safe_load(original)
    for name, layer in [('local_costmap','voxel_layer'),('global_costmap','obstacle_layer')]:
        p = expected[name][name]['ros__parameters'][layer]['scan']
        p['topic'] = NAV_SCAN_TOPIC
        p['expected_update_rate'] = .5
        params = expected[name][name]['ros__parameters']
        params['plugins'].append('body_clear_layer')
        params['body_clear_layer'] = {'plugin': 'raicom_nav2_workspace::BodyClearLayer'}
    assert generated == expected
    assert footprint == BODY
    assert source.read_bytes() == original
    assert navigation_scan_profile(source,tmp_path)[0] == path
