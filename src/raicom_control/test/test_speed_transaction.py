"""Real parameter transaction, mocked ROS transport; no runtime writes."""
from concurrent.futures import Future
from copy import deepcopy
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from raicom_control.field_tool_speed import NAMES,ProfileReadback
from raicom_control.speed_limits import GUARD_SPEED_CONTRACT,guard_targets


def harness(monkeypatch,guard=.5,refuse=None):
    monkeypatch.setitem(sys.modules,'rcl_interfaces.srv',NS(GetParameters=NS(Request=lambda **k:NS(**k)),
                        SetParametersAtomically=NS(Request=lambda **k:NS(**k))))
    monkeypatch.setitem(sys.modules,'rclpy.parameter',NS(Parameter=lambda name,value:
                        NS(to_parameter_msg=lambda:NS(name=name,value=value))))
    values={'controller_server':['raicom_nav2_workspace::PhasedDwbController',.25,.12,.03,.25,.25,.25,.25,0.,.02,math.radians(10),False,.02,.35,.35,.5,-.5,.7,-.7],
            'velocity_smoother':[[.25,0.,.35],[-.08,0.,-.35],[.5,0.,.7],[-.5,0.,-.7]],
            'cmd_vel_guard':[guard,.5,GUARD_SPEED_CONTRACT,.5,.7]}
    state={k:dict(zip(NAMES[k],deepcopy(v))) for k,v in values.items()}
    writes=[];clients=[]
    def encode(v):
        if type(v) is bool:return NS(type=1,bool_value=v)
        if isinstance(v,str):return NS(type=4,string_value=v)
        if isinstance(v,list):return NS(type=8,double_array_value=v)
        return NS(type=3,double_value=v)
    def create(_kind,path):
        name=path.split('/')[1];client=Mock();clients.append(client)
        def call(request):
            f=Future()
            if path.endswith('/get_parameters'):
                f.set_result(NS(values=[encode(state[name][k]) for k in request.names]))
            else:
                writes.append((name,{p.name:p.value for p in request.parameters}))
                if name!=refuse:state[name].update(writes[-1][1])
                f.set_result(NS(result=NS(successful=name!=refuse)))
            return f
        client.call_async.side_effect=call;client.service_is_ready.return_value=True
        return client
    node=Mock();node.create_client.side_effect=create;node.zero_velocity_hold_ok.return_value=True
    return node,state,writes,clients


def finish(gate):
    for _ in range(20):
        result=gate.poll()
        if result is not None:return result
    pytest.fail('transaction did not finish')


@pytest.mark.parametrize('speed,turn',[(.18,.05),(.25,.35),(.35,.8),(.5,1.2),(1.2,.3),(2.,2.)])
def test_all_three_limits_apply_before_verified_result_acceleration_unchanged(monkeypatch,speed,turn):
    node,state,writes,clients=harness(monkeypatch)
    original=deepcopy(state['cmd_vel_guard'])
    config={'far':speed,'turn':turn};gate=ProfileReadback(node,config)
    result=finish(gate)
    assert result['far']==result['cap']==speed
    assert result['turn']==turn
    assert [n for n,_ in writes]==['controller_server','velocity_smoother','cmd_vel_guard']
    assert state['controller_server']['FollowPath.max_speed_xy']==speed
    assert state['controller_server']['FollowPath.max_vel_x']==speed
    assert state['controller_server']['FollowPath.turn_speed']==turn
    assert state['controller_server']['FollowPath.max_vel_theta']==turn
    assert state['velocity_smoother']['max_velocity']==[speed,0.,turn]
    assert state['velocity_smoother']['min_velocity']==[-.08,0.,-turn]
    original.update(dict(zip(('max_linear','max_angular'),guard_targets(config))))
    assert state['cmd_vel_guard']==original
    gate.close();gate.close();assert node.destroy_client.call_count==6


@pytest.mark.parametrize('config',[{'far':2.01},{'turn':2.01},{'turn':0.},{'turn':float('nan')},
                                   {'turn':True},{'far':float('inf')},{'near':2.01},{'fine':2.01},
                                   {'near_distance':-.01},{'near_distance':float('nan')}])
def test_over_software_max_or_invalid_input_rejected_before_any_write(monkeypatch,config):
    node,state,writes,_=harness(monkeypatch)
    with pytest.raises(ValueError):ProfileReadback(node,config)
    assert not writes


@pytest.mark.parametrize('changes',[
    {'far':2.,'near':1.,'fine':.5,'near_distance':.1,'turn':1.},
    {'far':.25,'near':1.2,'fine':.5,'near_distance':.03},
    {'far':.25,'near':.12,'fine':.5,'near_distance':0.},
    {'fine':.005,'near_distance':3.,'turn':.01},
])
def test_independent_stage_values_and_distance_are_applied_without_recommendation_gate(monkeypatch,changes):
    from raicom_control.field_tool_speed import speed_settings
    node,state,writes,_=harness(monkeypatch)
    config=speed_settings(changes);gate=ProfileReadback(node,config);result=finish(gate)
    for key in ('far','near','fine','turn'):assert result[key]==config[key]
    assert state['controller_server']['FollowPath.near_distance']==config['near_distance']
    maximum=max(config[k] for k in ('far','near','fine'))
    assert state['controller_server']['FollowPath.max_vel_x']==maximum
    assert state['velocity_smoother']['max_velocity'][0]==maximum
    assert state['cmd_vel_guard']['max_linear']==max(.5,maximum)
    gate.close()


def test_old_guard_contract_or_changed_acceleration_blocks_any_write(monkeypatch):
    for key,value in [('speed_contract_version','old'),('max_linear_accel',.1),('max_angular_accel',2.)]:
        node,state,writes,_=harness(monkeypatch)
        state['cmd_vel_guard'][key]=value
        gate=ProfileReadback(node,{'far':2.,'turn':2.})
        with pytest.raises(ValueError):finish(gate)
        assert not writes and gate.result is None


@pytest.mark.parametrize('refuse',['controller_server','velocity_smoother','cmd_vel_guard'])
def test_any_writer_rejection_cannot_return_success_and_requires_reapply(monkeypatch,refuse):
    node,state,writes,_=harness(monkeypatch,refuse=refuse)
    gate=ProfileReadback(node,{'far':.35})
    with pytest.raises(RuntimeError,match=refuse):finish(gate)
    assert gate.result is None and writes[-1][0]==refuse
    # Cross-node operations are not atomic; controller may have changed.
    assert state['controller_server']['FollowPath.max_vel_x']==(.25 if refuse=='controller_server' else .35)
    gate.close()


def test_moving_and_deadline_block_new_writes(monkeypatch):
    node,state,writes,_=harness(monkeypatch)
    gate=ProfileReadback(node,{'far':.35});node.zero_velocity_hold_ok.return_value=False
    with pytest.raises(RuntimeError,match='停稳'):gate.poll()
    assert not writes
    node.zero_velocity_hold_ok.return_value=True;gate.deadline=0.
    with pytest.raises(RuntimeError,match='超时'):gate.poll()
    assert not writes


def test_runtime_change_during_readback_is_detected(monkeypatch):
    node,state,writes,_=harness(monkeypatch)
    gate=ProfileReadback(node,{'far':.35})
    for _ in range(20):
        gate.poll()
        if gate.phase=='read':break
    state['velocity_smoother']['max_velocity'][0]=.25
    with pytest.raises(ValueError,match='同时'):finish(gate)
    assert gate.result is None


def test_cancel_removes_pending_requests(monkeypatch):
    node,state,writes,clients=harness(monkeypatch)
    gate=ProfileReadback(node,{'far':.35})
    pending=[]
    for client in clients:
        f=Future();pending.append(f);client.call_async.side_effect=lambda _,f=f:f
    gate.poll();gate.close()
    assert all(f.cancelled() for f in pending[:3])
    assert node.destroy_client.call_count==6 and not writes


@pytest.mark.parametrize('index,phase',[(3,'set_controller'),(4,'set_smoother'),(5,'set_guard')])
def test_cancelled_write_must_settle_before_a_new_speed_transaction(monkeypatch,index,phase):
    node,state,writes,clients=harness(monkeypatch)
    first=ProfileReadback(node,{'far':.35})
    late=Future();clients[index].call_async.side_effect=lambda _:late
    for _ in range(12):first.poll()
    assert first.phase==phase and 'set' in first.pending
    first.close()
    assert not late.cancelled() and len(first.unsettled)==1
    second=ProfileReadback(node,{'far':.25})
    assert second.poll() is None
    for client in clients[6:]:client.call_async.assert_not_called()
    state['controller_server']['FollowPath.max_vel_x']=.35
    state['cmd_vel_guard']['max_linear']=2.
    late.set_result(NS(result=NS(successful=True)))
    assert finish(second)['far']==.25
    assert state['controller_server']['FollowPath.max_vel_x']==.25 and not second.unsettled
    assert state['cmd_vel_guard']['max_linear']==.5
    second.close()


@pytest.mark.parametrize('active',[False,True])
def test_ui_manual_apply_waits_active_and_never_starts_navigation(monkeypatch,active):
    from raicom_control.field_tool_workspace import WorkspaceMixin
    now=[0.];monkeypatch.setattr('raicom_control.field_tool_workspace.time.monotonic',lambda:now[0])
    class Var:
        def __init__(self,value):self.value=value
        def get(self):return self.value
        def set(self,value):self.value=value
    app=WorkspaceMixin();app._can_edit_route=lambda:True
    app.route_document={'map':{},'points':{},'order':[],'history':[]}
    app._workspace_write=lambda value:setattr(app,'route_document',value)
    app._workspace_snapshot=lambda:'fixed';app._lock_manual_for_task=Mock();app._set_status=Mock()
    app._workspace_journal=Mock();app.navigation_speed_var=Var('')
    app.speed_mode_var=Var('分阶段前向接近')
    app.navigation_speed_vars={k:Var(v) for k,v in dict(far='.35',near='.12',fine='.03',near_distance='.25').items()}
    app.node=NS(navigation=NS(cleanup_pending=False),zero_velocity_hold_ok=lambda:True,
                reset_navigation_readiness=Mock(),navigation_ready=lambda:active)
    gate=Mock();gate.poll.return_value={'far':.35,'near':.12,'fine':.03,'turn':.35}
    factory=Mock(return_value=gate);monkeypatch.setattr('raicom_control.field_tool_workspace.ProfileReadback',factory)
    app.save_navigation_speeds();assert app._workspace_busy()
    now[0]=4.;app._poll_speed_apply()
    assert app.speed_apply is None and app.route_document['workspace']['speeds']['far']==.35
    if active:
        gate.close.assert_called_once();assert '已生效回读' in app.navigation_speed_var.get()
    else:
        factory.assert_not_called();assert '尚未应用' in app.navigation_speed_var.get()
