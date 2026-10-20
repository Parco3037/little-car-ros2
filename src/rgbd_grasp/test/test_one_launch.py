"""Regression checks for startup, cancellation and bus contention (no hardware)."""
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import threading
import pytest
from rgbd_grasp.grasp_core import Rejected, stable_target, select_candidate_uvz
from rgbd_grasp.grasp_once_node import GraspOnceNode
from rgbd_grasp.grasp_hardware import GraspHardware
from rgbd_grasp.target_to_ik_node import TargetToIkNode


def test_full_automatic_order_once():
    order=[]
    n=NS(holding=False,execution_lock=threading.Lock(),cancel_event=threading.Event(),
         parameters={'auto_grasp':True,'auto_target_wait_s':8.0},
         hardware=NS(stop=Mock()),publish_status=lambda s,*a,**kw:order.append(s),
         move_to_initial_pose=lambda: (order.append('pose') or {1:450}),
         wait_for_target=lambda t:(order.append('detect') or True),
         acquire_target_with_search=lambda:order.append('search'),
         center_target_with_id1=lambda:order.append('align'),
         run_precheck=lambda:(order.append('plan') or {}),
         execute=lambda p:order.append('grip'))
    GraspOnceNode.auto_prepare_worker(n)
    assert [x for x in order if x in ('pose','detect','align','plan','grip')] == ['pose','detect','align','plan','grip']
    assert order[-1]=='auto_grasp_complete'
    assert not n.execution_lock.locked()


def test_stop_cancels_pending_start():
    n=NS(cancel_event=threading.Event(),auto_prepare_timer=Mock(),hardware=NS(stop=Mock()),
         publish_status=Mock(),finish=lambda r,s,m:s)
    timer=n.auto_prepare_timer
    assert GraspOnceNode.stop_callback(n,None,NS())
    assert n.cancel_event.is_set() and n.auto_prepare_timer is None
    timer.cancel.assert_called_once()


def test_stop_before_worker_cannot_be_cleared_by_startup():
    event=threading.Event();event.set()
    n=NS(cancel_event=event,publish_status=Mock())
    GraspOnceNode.auto_prepare_worker(n)
    assert event.is_set()
    assert n.publish_status.call_args.args[0]=='cancelled'


@pytest.mark.parametrize('stage', ['prepare','centering','search','precheck','approach','close_gripper','rejected','stopped'])
def test_motion_stages_cancel_perception_reads(stage):
    future=Mock()
    n=NS(observation_enabled=True,pending=future,pending_message=object(),latest_message=object())
    TargetToIkNode.stage_callback(n,NS(data=json.dumps({'stage':stage})))
    assert not n.observation_enabled and n.pending is None and n.latest_message is None
    future.cancel.assert_called_once()


def test_resuming_perception_requires_post_motion_frame():
    n=NS(observation_enabled=False,latest_message=object(),pending=None,
         get_clock=lambda:NS(now=lambda:NS(nanoseconds=5000000000)))
    TargetToIkNode.stage_callback(n,NS(data='{"stage":"observing"}'))
    assert n.observation_enabled and n.observation_after_s==5 and n.latest_message is None


def test_duplicate_named_controller_is_rejected():
    own=NS(node_name='grasp_once_node',node_namespace='/')
    n=NS(get_publishers_info_by_topic=lambda t:[own,own],get_name=lambda:'grasp_once_node',get_namespace=lambda:'/')
    with pytest.raises(Rejected,match='duplicate'):
        GraspOnceNode.assert_single_command_source(n)


@pytest.mark.parametrize('stamps', [[1,1,1],[1,1.2,1.1],[1,9.9,10]])
def test_replayed_or_old_frames_do_not_make_stable_target(stamps):
    samples=[{'stamp_s':s,'frame':'arm','point':[.2,0,.2]} for s in stamps]
    with pytest.raises(Rejected):
        stable_target(samples,10.1,'arm',3,1,.005,[0,.35,-.2,.2,.08,.55])


def test_wait_does_not_accept_stationary_read_until_command_elapsed():
    clock=[0.0]; reads=[]
    n=NS(parameters={'settle_timeout_s':3.,'settle_tolerance':30.,'stable_joint_samples':2})
    h=GraspHardware.__new__(GraspHardware);h.node=n;h.cancelled=lambda:False;h.command_end=2.
    h.read=lambda ids:(reads.append(clock[0]) or {1:450,5:860})
    h.assert_fixed_id5=lambda measured:None
    with patch('rgbd_grasp.grasp_hardware.time.monotonic',lambda:clock[0]),patch('rgbd_grasp.grasp_hardware.time.sleep',lambda dt:clock.__setitem__(0,clock[0]+dt)):
        assert h.wait((1,),[450])[1]==450
    assert reads[0]>=2. and len(reads)>=3


@pytest.mark.parametrize('values', [[300.,330.,.2,345.,230.,.24], [345.,230.,.24,300.,330.,.2]])
def test_upper_cube_selected_independent_of_contour_order(values):
    selected,count=select_candidate_uvz(values)
    assert selected.tolist()==[345.,230.,.24] and count==2


def test_centers_the_upper_cube_with_multiple_candidates():
    n=NS(parameters={'target_selection':'min_v','require_single_candidate':False,'candidate_max_age_s':3.},
         get_clock=lambda:NS(now=lambda:NS(nanoseconds=5000000000)))
    GraspOnceNode.candidate_callback(n,NS(data=[300.,330.,.2,345.,230.,.24]))
    assert GraspOnceNode.current_candidate_u(n)==345.
    n.parameters['require_single_candidate']=True
    with pytest.raises(Rejected,match='exactly one'):
        GraspOnceNode.current_candidate_u(n)


@pytest.mark.parametrize('values', [[], [1,2], [1,2,-1], [1,float('nan'),.2]])
def test_bad_candidates_cannot_select_a_cube(values):
    with pytest.raises(Rejected):select_candidate_uvz(values)
