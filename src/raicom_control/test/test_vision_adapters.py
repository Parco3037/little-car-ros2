"""No ROS runtime, network or hardware access; exercise actual adapter boundaries."""
from concurrent.futures import Future
from contextlib import nullcontext
from copy import deepcopy
import io
import json
from types import ModuleType,SimpleNamespace as NS
import sys
from unittest.mock import Mock

import pytest
from raicom_control.station_mapping import StationMapping
from raicom_control.vision_http import VisionBackend,request_bytes,valid_request
from raicom_control.vision_ros_client import VisionClient


def frame(swap=False):
    return {'payload':{'status':'success','image_size':[640,480],'detections':[
        {'class':'2' if swap else '1','score':.9,'bbox':[10,20,100,120]},
        {'class':'1' if swap else '2','score':.9,'bbox':[310,20,400,120]}]}}


@pytest.mark.parametrize('swap',[False,True])
def test_three_frame_mapping_uses_current_point_names_and_swapped_sides(swap):
    session=StationMapping('task-a','map-and-points','shibie',{'A':'park_left','B':'park_right'})
    for _ in range(2):assert session.observe(frame(swap),stopped=True) is None
    result=session.observe(frame(swap),stopped=True)
    assert result['category_to_park']==({'G':'A','T':'B'} if swap else {'T':'A','G':'B'})
    assert result['category_to_point']['T']==('park_right' if swap else 'park_left')
    assert result['task_id']=='task-a' and result['geometry_identity']=='map-and-points'
    assert result['confirmed_frames']==3 and result['label_to_park']['1']==result['category_to_park']['T']
    another=StationMapping('task-b','map-and-points','shibie',session.destinations)
    assert another.observe(frame(swap),stopped=True) is None


@pytest.mark.parametrize('fault',['empty','unknown','failure','moving','swapped'])
def test_invalid_or_changed_frame_resets_streak(fault):
    session=StationMapping('task','geometry','station',{'A':'p1','B':'p2'})
    for _ in range(2):session.observe(frame(),stopped=True)
    value=frame()
    if fault=='empty':value['payload']['detections']=[]
    if fault=='unknown':value['payload']['detections'][0]['class']='other'
    if fault=='failure':value={}
    if fault=='swapped':value=frame(True)
    assert session.observe(value,stopped=fault!='moving') is None
    for _ in range(2):assert session.observe(frame(),stopped=True) is None
    assert session.observe(frame(),stopped=True)['valid']


def test_blue_is_not_a_delivery_category():
    session=StationMapping('task','geometry','station',{'A':'p1','B':'p2'})
    value=frame();value['payload']['detections'].append({'class':'blue'})
    for _ in range(3):result=session.observe(value,stopped=True)
    assert set(result['category_to_park'])=={'T','G'}


def test_backend_inference_is_single_no_body_post_and_health_is_read_only():
    fetch=Mock(side_effect=[json.dumps(frame()['payload']).encode(),b'\xff\xd8jpeg'])
    backend=VisionBackend(fetch=fetch,lock=nullcontext)
    result=backend.perform('infer')
    assert result['payload']['detections']==frame()['payload']['detections'] and result['jpeg']==b'\xff\xd8jpeg'
    assert fetch.call_count==2
    assert fetch.call_args_list[0].args==('http://127.0.0.1:8080/api/do_infer','POST')
    assert fetch.call_args_list[1].args==('http://127.0.0.1:8000/api/temp_image',)
    fetch.reset_mock();fetch.side_effect=None;status=backend.health('camera')
    assert status['backend_reachable'] and not status['recognition_validated']
    fetch.assert_called_once_with('http://127.0.0.1:8000/api/params',timeout=2.,limit=200_000)
    fetch.side_effect=TimeoutError('connection timed out')
    assert not backend.health('infer')['backend_reachable']


def test_camera_fetches_verified_jpeg_endpoint_and_does_not_open_device():
    fetch=Mock(side_effect=[b'{"status":"success"}',b'\xff\xd8jpeg'])
    result=VisionBackend(fetch=fetch,lock=nullcontext).perform('camera')
    assert result['jpeg']==b'\xff\xd8jpeg'
    assert fetch.call_args_list[1].args==('http://127.0.0.1:8000/api/temp_image',)


@pytest.mark.parametrize('response',[b'[]',b'{"status":"failed"}',b'{"status":"success"}',
                                     b'{"status":"success","detections":[],"value":NaN}'])
def test_http_200_business_malformed_and_nonfinite_failures_are_rejected(response):
    with pytest.raises((RuntimeError,ValueError)):
        VisionBackend(fetch=Mock(return_value=response),lock=nullcontext).perform('infer')


def test_request_has_no_body_and_bounded_response():
    opener=Mock(side_effect=lambda *_args,**_kw:io.BytesIO(b'12345'))
    with pytest.raises(ValueError):request_bytes('http://localhost/test','POST',limit=4,opener=opener)
    request=opener.call_args.args[0]
    assert request.method=='POST' and request.data is None


def fake_string(monkeypatch):
    package=ModuleType('std_msgs');msg=ModuleType('std_msgs.msg');msg.String=lambda **kw:NS(**kw)
    monkeypatch.setitem(sys.modules,'std_msgs',package);monkeypatch.setitem(sys.modules,'std_msgs.msg',msg)
    return msg.String


def test_ros_client_correlates_ignores_late_duplicate_and_cancels_timeout(monkeypatch):
    string=fake_string(monkeypatch)
    node=Mock();node.get_publishers_info_by_topic.return_value=[object()]
    pub=node.create_publisher.return_value;pub.get_subscription_count.return_value=1
    client=VisionClient(node)
    future=client.request('task1');request=json.loads(pub.publish.call_args.args[0].data)
    value=dict(request,status='success',reply=frame())
    client.receive(string(data=json.dumps(dict(value,task_id='other'))));assert not future.done()
    client.receive(string(data=json.dumps(value)));assert future.result()==frame()
    other=client.request('task2');client.receive(string(data=json.dumps(value)));assert not other.done()
    client.cancel();assert other.cancelled() and client.pending is None
    client.receive(string(data=json.dumps(value)));assert other.cancelled()
    expired=client.request('task3');request,f,deadline=client.pending;client.pending=(request,f,0.)
    client.poll()
    with pytest.raises(TimeoutError):expired.result()
    assert json.loads(pub.publish.call_args.args[0].data)['command']=='cancel'
    client.close()
    node.destroy_subscription.assert_called_once();node.destroy_publisher.assert_called_once()


def test_multiple_infer_nodes_block_a_new_request(monkeypatch):
    fake_string(monkeypatch);node=Mock();node.get_publishers_info_by_topic.return_value=[1,2]
    node.create_publisher.return_value.get_subscription_count.return_value=1
    client=VisionClient(node)
    with pytest.raises(RuntimeError):client.request('task')
    node.create_publisher.return_value.publish.assert_not_called()


def test_visual_nodes_can_start_during_recognition_wait_without_starting_docker():
    from raicom_control.field_tool_delivery_ui import DeliveryWorkspaceMixin
    app=DeliveryWorkspaceMixin();app.node=NS(get_node_names=lambda:[])
    app._can_edit_route=lambda:False;app._launch=Mock();app._mapping_progress=Mock()
    app.start_vision_nodes()
    app._launch.assert_called_once_with('vision_http',['ros2','launch','raicom_control','vision_http.launch.py'])
    app.node.get_node_names=lambda:['yolo_camera'];app._launch.reset_mock()
    app.start_vision_nodes();app._launch.assert_not_called()


@pytest.mark.parametrize('value',[{},[],{'task_id':'t','request_id':'r','command':'drive'},
                                  {'task_id':'t','request_id':'','command':'start'}])
def test_invalid_ros_requests_are_not_executed(value):assert not valid_request(value)


def test_ros_node_trigger_returns_before_http_cancel_suppresses_image_and_consumes_id(monkeypatch):
    """Run the real node callbacks with ROS primitives stubbed, pending HTTP futures."""
    import raicom_control.vision_ros_nodes as nodes
    fake_string(monkeypatch)
    rclpy=ModuleType('rclpy');rclpy.init=Mock();rclpy.shutdown=Mock()
    node_module=ModuleType('rclpy.node')
    class Node:
        def __init__(self,name):self.name=name;self.params={};self.get_clock=Mock()
        def declare_parameter(self,k,v):self.params[k]=v
        def get_parameter(self,k):return NS(value=self.params[k])
        def create_publisher(self,*_):return Mock()
        def create_subscription(self,*_):return Mock()
        def create_service(self,*_):return Mock()
        def create_timer(self,*_):return Mock()
        def get_name(self):return self.name
        def destroy_node(self):pass
    node_module.Node=Node
    for name,module in [('rclpy',rclpy),('rclpy.node',node_module),('std_srvs.srv',NS(Trigger=object)),
                        ('sensor_msgs.msg',NS(CompressedImage=lambda:NS(header=NS())))]:
        monkeypatch.setitem(sys.modules,name,module)
    futures=[]
    pool=Mock()
    def submit(*_args):f=Future();futures.append(f);return f
    pool.submit.side_effect=submit;monkeypatch.setattr(nodes,'ThreadPoolExecutor',lambda **_:pool)
    def spin(node):
        response=node.trigger(None,NS());assert response.success
        request=json.loads(response.message)
        assert not futures[0].done() and node.result_pub.publish.call_count==0
        node.request(NS(data=json.dumps(request)))  # No second HTTP operation.
        assert len(futures)==1 and node.result_pub.publish.call_count==0
        node.request(NS(data=json.dumps(dict(request,command='cancel'))))
        futures[0].set_result({'jpeg':b'\xff\xd8mock','payload':{'status':'success'}})
        node.poll()
        assert json.loads(node.result_pub.publish.call_args.args[0].data)['status']=='cancelled'
        node.image_pub.publish.assert_not_called()
        node.request(NS(data=json.dumps(request)))
        assert len(futures)==1
        assert json.loads(node.result_pub.publish.call_args.args[0].data)['status']=='error'
        node.health_at=0.;node.poll();assert len(futures)==2
        futures[1].set_result({'backend_reachable':False,'recognition_validated':False})
        node.poll()
        assert not json.loads(node.status_pub.publish.call_args.args[0].data)['backend_reachable']
    rclpy.spin=spin
    nodes.camera_main([])
    pool.shutdown.assert_called_once_with(wait=False,cancel_futures=True)
