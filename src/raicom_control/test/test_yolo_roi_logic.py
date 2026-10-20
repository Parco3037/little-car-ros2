import pytest

from raicom_control.yolo_roi_logic import (
    Detection,
    choose_detection,
    detections_in_normalized_roi,
    normalized_roi,
)


def test_ambiguous_detection_is_rejected():
    detections = [
        Detection('1', 0.9, (10, 20, 50, 80)),
        Detection('2', 0.8, (60, 20, 100, 80)),
    ]
    assert choose_detection(detections) is None
    assert choose_detection(detections, desired_label='1').label == '1'


def test_roi_is_normalized_for_different_camera_sizes():
    target = Detection('1', 0.9, (100, 50, 300, 250))
    roi = normalized_roi(target, 640, 400)
    assert roi['space'] == 'normalized'
    assert roi['x'] == 0.15625
    assert roi['y'] == 0.125
    assert roi['w'] == 0.3125
    assert roi['h'] == 0.5


def test_fixed_scene_roi_reduces_two_boxes_to_one_candidate():
    detections = [
        Detection('1', 0.9, (100, 100, 200, 200)),
        Detection('2', 0.8, (400, 100, 500, 200)),
    ]
    selected = detections_in_normalized_roi(
        detections, (0.0, 0.0, 0.5, 1.0), image_width=600, image_height=400
    )
    assert [d.label for d in selected] == ['1']
    assert choose_detection(selected).label == '1'


def test_fixed_scene_roi_is_fail_closed_when_invalid():
    with pytest.raises(ValueError, match='exceeds'):
        detections_in_normalized_roi(
            [], (0.8, 0.0, 0.4, 1.0), image_width=600, image_height=400
        )
