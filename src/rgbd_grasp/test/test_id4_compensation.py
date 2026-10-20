"""Command-space compensation does not change nominal IK or the carry pose."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import pytest

from rgbd_grasp.grasp_core import (
    Rejected, compensate_grasp_id4, validate_commanded_fk,
)
from rgbd_grasp.grasp_hardware import GraspHardware
from rgbd_grasp.grasp_once_node import GraspOnceNode


@pytest.mark.parametrize('offset', [0, 30, 60])
def test_only_id4_changes_and_input_is_not_mutated(offset):
    nominal = np.array([450, 600, 200, 350, 860])
    adjusted = compensate_grasp_id4(nominal, offset, [42,958])
    assert adjusted.tolist() == [450,600,200,350+offset,860]
    assert nominal.tolist() == [450,600,200,350,860]


@pytest.mark.parametrize('offset', [-1, 61, 1.5, float('nan'), float('inf')])
def test_invalid_compensation_is_rejected(offset):
    with pytest.raises(Rejected, match='grasp_id4_offset_pulse'):
        compensate_grasp_id4([450,600,200,350,860], offset, [42,958])


def test_joint_limit_crossing_is_rejected_instead_of_clamped():
    with pytest.raises(Rejected, match='outside servo limits'):
        compensate_grasp_id4([450,600,200,910,860], 60, [42,958])


@pytest.mark.parametrize('xyz', [[.36,0,.2], [.2,0,.56], [.2,float('nan'),.2]])
def test_compensated_fk_must_stay_in_workspace(xyz):
    with pytest.raises(Rejected):
        validate_commanded_fk(xyz, [0,.35,-.2,.2,.08,.55])


def planning_fixture(offset=60):
    params = {
        'require_single_candidate':False, 'visual_center_enabled':False,
        'grasp_height_offset_m':.015, 'grasp_id4_offset_pulse':offset,
        'grasp_offset':[0.,0.,0.], 'approach_distance_m':.05,
        'lift_distance_m':.03, 'cartesian_step_m':.005,
        'servo_limits':[[350,550],[125,875],[0,1000],[42,958],[0,1000]],
        'id1_range':[350,550], 'fixed_id5':860,
        'max_first_joint_jump':450, 'max_path_joint_jump':80,
        'max_carry_joint_jump':700, 'carry_pose':[450,500,250,400,860],
        'workspace':[0,.35,-.2,.2,.08,.55],
    }
    hardware = Mock(spec=GraspHardware)
    hardware.read.return_value = {1:450,2:800,3:50,4:300,5:860,10:390}
    hardware.ik.side_effect = [np.array([450,600,200,300+i*10,860]) for i in range(3)]
    hardware.fk.return_value = np.array([.20,0.,.25])
    node = NS(parameters=params, hardware=hardware, assert_single_command_source=Mock(),
              current_candidate_u=Mock(), current_target=lambda:np.array([.2,0,.2]),
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=10**9)), last_candidate_count=2)
    return node


@pytest.mark.parametrize('offset', [0,60])
def test_planning_compensates_each_approach_once_but_leaves_carry(offset):
    node = planning_fixture(offset)
    waypoints = ([.15,0,.215], [[.175,0,.215],[.2,0,.215]], [])
    with patch('rgbd_grasp.grasp_once_node.side_grasp_waypoints', return_value=waypoints):
        plan = GraspOnceNode.plan(node)
    assert plan['pregrasp'][3] == 300+offset
    assert [p[3] for p in plan['approach']] == [310+offset,320+offset]
    assert plan['carry'].tolist() == [450,500,250,400,860]
    assert plan['nominal_final_pulses'][3] == 320
    assert plan['commanded_final_pulses'][3] == 320+offset
    assert node.hardware.ik.call_count == node.hardware.fk.call_count == 3
    node.hardware.command.assert_not_called()


def test_nominal_fk_error_is_still_rejected():
    hardware = GraspHardware.__new__(GraspHardware)
    hardware.node = NS(parameters=dict(planning_fixture().parameters,
        pitch_deg=30.,pitch_range_deg=[-180.,180.]))
    hardware.cancelled = lambda:False
    hardware.ik_client = object()
    hardware.call = Mock(return_value=NS(pulse=[450,600,200,300,860]))
    hardware.fk = Mock(return_value=np.array([.22,0,.2]))
    with pytest.raises(Rejected, match='IK/FK position error exceeds 5mm'):
        hardware.ik([.2,0,.2])
