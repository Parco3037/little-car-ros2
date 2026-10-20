"""Execute actual guard callbacks without ROS imports, network or actuators."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from raicom_control.guard_logic import AccelerationLimiter,MotionEnableLease,WatchdogState,ZERO_COMMAND,sanitize_command
from raicom_control.speed_limits import GUARD_SPEED_CONTRACT,validate_guard_limits

ROOT=Path(__file__).parents[1]


def guard_fixture():
    clock=[100.]
    tree=ast.parse((ROOT/'raicom_control/cmd_vel_guard.py').read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CmdVelGuard')
    cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name!='__init__']
    def twist():return NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))
    namespace=dict(Node=object,Twist=twist,time=NS(monotonic=lambda:clock[0]),json=__import__('json'),
                   validate_guard_limits=validate_guard_limits,SetParametersResult=lambda **kw:NS(**kw),
                   ZERO_COMMAND=ZERO_COMMAND,sanitize_command=sanitize_command)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'<actual-guard-callbacks>','exec'),namespace)
    guard=namespace['CmdVelGuard']()
    parameters=dict(max_linear=.5,max_angular=.5,timeout_ms=150,max_linear_accel=.5,max_angular_accel=.7,
                    speed_contract_version=GUARD_SPEED_CONTRACT)
    guard.get_parameter=lambda name:NS(value=parameters[name])
    guard.get_logger=lambda:Mock()
    guard.state=WatchdogState(.15);guard.acceleration=AccelerationLimiter(.5,.7)
    guard.motion_enable=MotionEnableLease();guard.stop_latched=False;guard.publisher=Mock();guard._stale_reported=False
    def apply(changes,other_callback_rejects=False):
        result=guard._validate_speed_parameters([NS(name=k,value=v) for k,v in changes.items()])
        if result.successful and not other_callback_rejects:parameters.update(changes)
        return result
    def enable():guard.motion_enable.receive(dict(boot_id='mock',mission_id='mock',enabled=True,expires_at=clock[0]+.25),clock[0])
    return guard,parameters,clock,apply,enable,twist


def test_actual_guard_uses_committed_limits_not_validation_callback_side_effects():
    guard,params,clock,apply,enable,twist=guard_fixture()
    assert apply({'max_linear':2.,'max_angular':1.2},other_callback_rejects=True).successful
    assert guard.max_linear==.5 and guard.max_angular==.5
    assert apply({'max_linear':2.,'max_angular':1.2}).successful
    assert guard.max_linear==2. and guard.max_angular==1.2
    msg=twist();msg.linear.x=3.;msg.angular.z=3.
    output=[]
    for _ in range(100):
        enable();guard._on_command(msg);guard._publish_guarded()
        value=guard.publisher.publish.call_args.args[0];output.append((value.linear.x,value.angular.z))
        clock[0]+=.05
    assert output[-1]==(2.,1.2)
    assert all(0<=x<=2. and 0<=w<=1.2 for x,w in output)
    assert all(b[0]-a[0]<=.02500001 and b[1]-a[1]<=.03500001 for a,b in zip(output,output[1:]))
    # Actual speed protection remains independent of the selected ceiling.
    clock[0]+=.3;guard._publish_guarded()
    zero=guard.publisher.publish.call_args.args[0]
    assert zero.linear.x==zero.angular.z==0.


@pytest.mark.parametrize('changes',[{'max_linear':2.01},{'max_angular':2.01},{'max_linear':0.},
                                    {'max_angular':float('nan')},{'max_linear':float('inf')},
                                    {'max_angular':True},{'timeout_ms':999},{'max_linear_accel':2.}])
def test_out_of_range_nonfinite_and_unrelated_protection_changes_rejected(changes):
    guard,params,clock,apply,enable,twist=guard_fixture();before=dict(params)
    assert not apply(changes).successful and params==before
    assert guard.max_linear==guard.max_angular==.5


@pytest.mark.parametrize('field',['lease','command','ramp'])
def test_any_active_motion_blocks_upward_and_downward_limit_changes(field):
    from raicom_control.guard_logic import VelocityCommand
    guard,params,clock,apply,enable,twist=guard_fixture()
    if field=='lease':enable()
    if field=='command':guard.state.update(VelocityCommand(.1),clock[0])
    if field=='ramp':guard.acceleration.previous=VelocityCommand(.1)
    for limit in (.1,2.):assert not apply({'max_linear':limit,'max_angular':limit}).successful
    assert params['max_linear']==params['max_angular']==.5


def test_raise_limits_does_not_enable_motion_or_clear_stop_latch():
    guard,params,clock,apply,enable,twist=guard_fixture()
    guard._on_stop(None)
    assert apply({'max_linear':2.,'max_angular':2.}).successful
    assert guard.stop_latched and not guard._motion_enabled()
    msg=twist();msg.linear.x=2.;msg.angular.z=2.
    for _ in range(8):
        guard._on_command(msg);guard._publish_guarded();clock[0]+=.05
        output=guard.publisher.publish.call_args.args[0]
        assert output.linear.x==output.angular.z==0.


def test_manual_keyboard_remains_original_speed_with_higher_guard_ceiling():
    tree=ast.parse((ROOT/'raicom_control/field_tool_ros.py').read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FieldToolNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='manual_health_error')
    namespace={'time':NS(monotonic=lambda:10.)}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<actual-manual-health>','exec'),namespace)
    node=NS(ui_snapshot=dict(at=10.,base_ok=True,scan_ok=True,controls_unique=True,nav_bridge_present=False),
            control_parameters={'keyboard_input':(.25,.5,10.),'cmd_vel_guard':(2.,2.,10.)},
            manual_linear_speed=.25,manual_angular_speed=.5)
    assert namespace['manual_health_error'](node)==''
    node.control_parameters['cmd_vel_guard']=(2.01,2.,10.)
    assert namespace['manual_health_error'](node)
