"""Pairing and display semantics; no physical commands or camera access."""
from contextlib import contextmanager
import hashlib
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from raicom_control.vision_http import VisionBackend
from raicom_control.vision_ros_client import VisionClient
from raicom_control.vision_preview import annotations
from test_vision_adapters import fake_string,frame


def test_input_photo_is_fetched_before_releasing_inference_lock():
    locked=[]
    @contextmanager
    def lock():
        locked.append(True)
        try:yield
        finally:locked.pop()
    def fetch(url,*args,**kw):
        assert locked
        return json.dumps(frame()['payload']).encode() if args else b'\xff\xd8input'
    result=VisionBackend(fetch=fetch,lock=lock).perform('infer')
    assert result['jpeg']==b'\xff\xd8input' and not locked


def test_preview_failure_does_not_discard_successful_classification():
    fetch=Mock(side_effect=[json.dumps(frame()['payload']).encode(),TimeoutError('photo unavailable')])
    @contextmanager
    def lock():yield
    result=VisionBackend(fetch=fetch,lock=lock).perform('infer')
    assert result['payload']==frame()['payload']
    assert 'jpeg' not in result and result['preview_error']=='photo unavailable'


@pytest.mark.parametrize('image_first',[True,False])
def test_image_and_result_must_match_same_request_and_hash(monkeypatch,image_first):
    string=fake_string(monkeypatch);node=Mock()
    node.create_publisher.return_value.get_subscription_count.return_value=1
    node.get_publishers_info_by_topic.return_value=[object()]
    client=VisionClient(node);future=client.request('task')
    request=json.loads(client.publisher.publish.call_args.args[0].data)
    jpeg=b'\xff\xd8same-frame';reply=frame()
    reply['preview']={'stamp':[1,2],'sha256':hashlib.sha256(jpeg).hexdigest()}
    result=string(data=json.dumps(dict(request,status='success',reply=reply)))
    def image(data=jpeg,stamp=(1,2)):
        return NS(format='jpeg',data=data,header=NS(stamp=NS(sec=stamp[0],nanosec=stamp[1])))
    client.receive_image(image(stamp=(0,1)))
    client.receive_image(image(b'\xff\xd8wrong'))
    assert client.take_preview() is None
    if image_first:client.receive_image(image())
    client.receive(result)
    assert future.result()==reply
    if not image_first:
        assert client.take_preview() is None
        client.receive_image(image())
    packet=client.take_preview()
    assert packet['request_id']==request['request_id'] and packet['jpeg']==jpeg
    assert client.take_preview() is None
    # Late image/result after cancellation cannot repaint an old task.
    newer=client.request('new');client.cancel()
    client.receive_image(image());client.receive(result)
    assert newer.cancelled() and client.take_preview() is None


@pytest.mark.parametrize('swap',[True,False])
def test_candidate_park_labels_follow_left_right_and_never_claim_adopted(swap):
    size,boxes,summary=annotations(frame(swap)['payload'])
    assert size==(640,480)
    assert boxes[0]['label'].startswith('G' if swap else 'T')
    assert '园区1 候选' in boxes[0]['label'] and '园区2 候选' in boxes[1]['label']
    assert '尚需三次一致' in summary


def test_empty_unknown_and_overlapping_boxes_never_claim_park_mapping():
    payload=frame()['payload'];payload['detections']=[]
    assert annotations(payload)[1]==[] and '未检测到' in annotations(payload)[2]
    payload=frame()['payload'];payload['detections'][0]['class']='blue'
    assert all('园区' not in b['label'] for b in annotations(payload)[1])
    payload=frame()['payload'];payload['detections'][0]['bbox']=[10,20,350,120]
    assert '不建立' in annotations(payload)[2]
    payload['image_size']=[800000,480]
    with pytest.raises(ValueError):annotations(payload)


def test_stop_preview_cancels_only_vision_and_clears_frame():
    from raicom_control.field_tool_delivery_ui import DeliveryWorkspaceMixin
    app=NS(vision_preview_active=True,vision_preview_future=object(),vision_client=Mock(),
           vision_preview_button=Mock(),vision_preview_status=Mock(),detection_preview=Mock())
    DeliveryWorkspaceMixin._stop_vision_preview(app,'stopped')
    assert not app.vision_preview_active and app.vision_preview_future is None
    app.vision_client.cancel.assert_called_once()
    app.detection_preview.clear.assert_called_once_with('stopped')


def test_cancel_transport_error_still_clears_preview_for_other_stop_work():
    from raicom_control.field_tool_delivery_ui import DeliveryWorkspaceMixin
    app=NS(vision_preview_active=True,vision_preview_future=object(),vision_client=Mock(),
           vision_preview_button=Mock(),vision_preview_status=Mock(),detection_preview=Mock())
    app.vision_client.cancel.side_effect=RuntimeError('publisher gone')
    DeliveryWorkspaceMixin._stop_vision_preview(app,'stopped')
    assert not app.vision_preview_active
    assert 'publisher gone' in app.detection_preview.clear.call_args.args[0]


def test_preview_needs_no_map_and_does_not_adopt_mapping_or_move_robot():
    from raicom_control.field_tool_delivery_ui import DeliveryWorkspaceMixin
    app=NS(vision_preview_active=False,route_document=None,_workspace_busy=lambda:False,
           navigation_running=False,nav_start_pending=False,_clear_station_mapping=Mock(),
           vision_preview_button=Mock(),vision_preview_status=Mock(),start_vision_nodes=Mock())
    DeliveryWorkspaceMixin.toggle_vision_preview(app)
    assert app.vision_preview_active and app.mapping_session is None
    assert app.vision_preview_task.startswith('visual-preview-')
    app.start_vision_nodes.assert_called_once()
