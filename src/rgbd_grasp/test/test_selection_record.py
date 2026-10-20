"""Exercise real RGB-D processing without creating ROS nodes or actuators."""
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest

from rgbd_grasp.blue_block_3d_node import BlueBlock3DNode


def camera_fixture(directory):
    logger = Mock()
    node = NS(
        lower_blue=np.array([95, 80, 50], np.uint8),
        upper_blue=np.array([135, 255, 255], np.uint8),
        min_contour_area=300., max_square_aspect_ratio=1.7,
        fx=420., fy=420., cx=320., cy=200., camera_frame='camera',
        target_selection='min_v', bridge=CvBridge(), frame_count=0,
        blocks_pub=Mock(), target_point_pub=Mock(), debug_pub=Mock(),
        selection_record_dir=str(directory), selection_recorded=False,
        latest_selection_frame=None, get_logger=lambda: logger,
        context=NS(get_domain_id=lambda: 124),
    )
    for name in ('detect_blue_blocks', 'get_depth_at_target', 'pixel_to_camera'):
        setattr(node, name, getattr(BlueBlock3DNode, name).__get__(node))
    return node


@pytest.mark.parametrize('upper_depth,expected_count,expected_v', [
    (220, 2, 150), (0, 1, 300)
])
def test_record_explains_upper_selection_or_depth_rejection(
        tmp_path, upper_depth, expected_count, expected_v):
    node = camera_fixture(tmp_path)
    rgb = np.zeros((400, 640, 3), np.uint8)
    depth = np.zeros((400, 640), np.uint16)
    for v, z in [(150, upper_depth), (300, 230)]:
        cv2.rectangle(rgb, (300, v-20), (340, v+20), (255, 0, 0), -1)
        depth[v-20:v+21, 300:341] = z
    rgb_msg = node.bridge.cv2_to_imgmsg(rgb, 'bgr8')
    depth_msg = node.bridge.cv2_to_imgmsg(depth, '16UC1')
    BlueBlock3DNode.image_callback(node, rgb_msg, depth_msg)
    payload = {'stage': 'ready', 'candidate_count': expected_count}
    ready = NS(data=json.dumps(payload))
    BlueBlock3DNode.grasp_status_callback(node, ready)
    BlueBlock3DNode.grasp_status_callback(node, ready)
    files = list(tmp_path.glob('domain_124/*/selection.json'))
    assert len(files) == 1  # repeated status must not fill storage
    record = json.loads(files[0].read_text())
    assert record['raw_candidate_count'] == 2
    assert record['valid_candidate_count'] == expected_count
    assert record['selected_uvz'][1] == expected_v
    assert record['ready_status'] == payload
    assert record['detections'][0]['rejected_reason'] == (
        None if upper_depth else 'no_valid_depth')
    assert np.array_equal(np.load(files[0].parent/'depth.npy'), depth)
    assert np.array_equal(cv2.imread(str(files[0].parent/'rgb.png')), rgb)
    assert node.target_point_pub.publish.call_count == 1


def test_record_io_failure_does_not_escape_callback(tmp_path):
    blocked = tmp_path/'file'
    blocked.write_text('not a directory')
    node = camera_fixture(blocked)
    frame = np.zeros((4, 4), np.uint8)
    node.latest_selection_frame = (frame, frame, frame, frame, {}, 0.)
    BlueBlock3DNode.grasp_status_callback(node, NS(data='{"stage":"ready"}'))
    node.get_logger().warning.assert_called_once()
    node.blocks_pub.publish.assert_not_called()
    node.target_point_pub.publish.assert_not_called()
