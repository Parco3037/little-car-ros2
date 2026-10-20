"""No physical hardware: hold pose, retained claw and failure boundaries."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from rgbd_grasp.grasp_core import Rejected
from rgbd_grasp.grasp_hardware import GraspHardware
from rgbd_grasp.grasp_once_node import GraspOnceNode


CARRY = [450, 500, 250, 400, 860]
VALUES = dict(zip((1, 2, 3, 4, 5), CARRY))


def hardware_fixture():
    hardware = GraspHardware.__new__(GraspHardware)
    hardware.node = NS(holding=True, assert_single_command_source=Mock(), parameters={
        'motion_enabled': True, 'carry_pose': CARRY, 'fixed_id5': 860,
        'motion_duration_s': .45, 'id1_range': [350, 550],
        'servo_limits': [[350,550], [125,875], [0,1000], [42,958], [0,1000]],
    })
    hardware.cancelled = lambda: False
    hardware.command_pub = Mock()
    return hardware


def test_hold_command_moves_all_five_joints_and_excludes_claw():
    hardware = hardware_fixture()
    hardware.command(VALUES, duration=.8, post_grasp_hold=True)
    message = hardware.command_pub.publish.call_args.args[0]
    assert {item.id: item.position for item in message.position} == VALUES
    assert message.duration == pytest.approx(.8)
    hardware.node.assert_single_command_source.assert_called_once()


@pytest.mark.parametrize('kwargs', [{}, {'allow_id5': True}])
def test_id5_still_forbidden_outside_hold(kwargs):
    hardware = hardware_fixture()
    with pytest.raises(Rejected, match='outside post-grasp hold'):
        hardware.command(VALUES, **kwargs)
    hardware.command_pub.publish.assert_not_called()


@pytest.mark.parametrize('values', [VALUES | {10:50},
                                   VALUES | {5:875}, {5:860}])
def test_hold_rejects_claw_command_or_unconfigured_pose(values):
    hardware = hardware_fixture()
    with pytest.raises(Rejected, match='match carry_pose'):
        hardware.command(values, post_grasp_hold=True)
    hardware.command_pub.publish.assert_not_called()


def test_hold_requires_retained_grasp_and_respects_stop():
    hardware = hardware_fixture()
    hardware.node.holding = False
    with pytest.raises(Rejected, match='requires a retained grasp'):
        hardware.command(VALUES, post_grasp_hold=True)
    hardware.node.holding = True
    hardware.cancelled = lambda: True
    with pytest.raises(Rejected, match='stopped by operator'):
        hardware.command(VALUES, post_grasp_hold=True)
    hardware.command_pub.publish.assert_not_called()


def execution_fixture(last_wait=None, empty=False):
    events = []
    hardware = Mock()
    hardware.wait.side_effect = [
        {10:390, 5:875}, {10:50 if empty else 180, 5:875},
        VALUES if last_wait is None else last_wait,
    ]
    node = NS(holding=False, hardware=hardware,
              parameters={'motion_enabled': True, 'gripper_open':390,
                          'gripper_close':50, 'carry_pose_duration_s':.8},
              ramp_to_pregrasp=Mock(), command_and_wait_arm=Mock(),
              publish_status=lambda stage, detail, **fields: events.append((stage,fields)),
              finish=lambda response, success, detail: (success,detail))
    plan = {'measured': {}, 'pregrasp': CARRY, 'approach': [], 'carry': CARRY}
    return node, plan, events


def test_execute_waits_for_five_joints_before_declaring_hold():
    node, plan, events = execution_fixture()
    GraspOnceNode.execute(node, plan)
    assert node.hardware.command.call_args_list[-1].args == (VALUES,)
    assert node.hardware.command.call_args_list[-1].kwargs == {
        'duration':.8, 'post_grasp_hold':True}
    assert node.hardware.wait.call_args.args == ((1,2,3,4,5), CARRY)
    assert node.holding and events[-1][0] == 'holding_unconfirmed'
    assert events[-1][1]['hold_pose_reached']
    assert events[-1][1]['measured'] == {str(sid):pulse for sid,pulse in VALUES.items()}


@pytest.mark.parametrize('reason', ['hardware settling timeout', 'stopped by operator'])
def test_hold_failure_keeps_claw_latched_and_prevents_new_grasp(reason):
    node, plan, events = execution_fixture(Rejected(reason))
    with pytest.raises(Rejected, match=reason):
        GraspOnceNode.execute(node, plan)
    assert node.holding
    assert not any(stage == 'holding_unconfirmed' for stage,_ in events)
    count = node.hardware.command.call_count
    success, detail = GraspOnceNode.grasp_callback(node, None, NS())
    assert not success and 'already holding' in detail
    assert node.hardware.command.call_count == count


def test_empty_close_never_moves_wrist_to_hold():
    node, plan, events = execution_fixture(empty=True)
    with pytest.raises(Rejected, match='empty-close'):
        GraspOnceNode.execute(node, plan)
    assert not node.holding
    assert node.hardware.command.call_count == 2
    assert not any(stage == 'move_to_hold_pose' for stage,_ in events)
