"""Read the current native grasp's motion evidence, never infer physical holding."""
from copy import deepcopy
import math

HOLD_POSE = [450, 620, 180, 260, 875]
HOLD_TOLERANCE = 30  # Unchanged native grasp settle_tolerance.


class GraspMotionEvidence:
    def __init__(self):
        self.ready = None
        self.hold = None
        self.tracking = None
        self.error = ''

    def observe(self, value):
        stage = value.get('stage')
        if stage == 'ready':
            self.ready = {k: deepcopy(value[k]) for k in (
                'candidate_count', 'selected_uvz', 'target_selection', 'target', 'grasp_target',
                'grasp_height_offset_m', 'grasp_id4_offset_pulse', 'nominal_final_pulses',
                'commanded_final_pulses', 'sag_compensation_m', 'pregrasp_clearance_m',
                'ramp_min_model_z_m') if k in value}
        if stage in {'move_to_pregrasp', 'approach'} and 'model_height_drop_m' in value:
            drop = value['model_height_drop_m']
            self.tracking = None
            if type(drop) in (int, float) and math.isfinite(drop):
                self.tracking = {k: deepcopy(value[k]) for k in (
                    'stage', 'planned', 'commanded', 'measured', 'reference_fk_xyz', 'commanded_fk_xyz',
                    'height_correction_attempt',
                    'measured_fk_xyz', 'model_height_drop_m', 'sag_compensation_m') if k in value}
        if stage == 'holding_unconfirmed':
            measured, grip = value.get('measured'), value.get('gripper_readback')
            valid = (value.get('hold_pose_reached') is True
                     and value.get('gripper_command_preserved') is True
                     and value.get('physical_holding_confirmed') is False
                     and value.get('goal') == HOLD_POSE
                     and isinstance(measured, dict) and set(measured) == {'1','2','3','4','5'}
                     and all(type(measured[str(i)]) is int and abs(measured[str(i)] - goal) <= HOLD_TOLERANCE
                             for i, goal in enumerate(HOLD_POSE, 1))
                     and type(grip) is int and 60 < grip <= 1000)
            if valid:
                self.hold = {k: deepcopy(value[k]) for k in ('goal','measured','gripper_readback',
                    'hold_pose_reached','gripper_command_preserved','physical_holding_confirmed')}
            else:
                self.hold = None
                self.error = '新版抓取缺少有效保持姿态/夹爪读回，不能确认动作完成'
        if stage == 'auto_grasp_complete' and (not self.hold or value.get('hold_pose_reached') is not True
                or value.get('gripper_command_preserved') is not True
                or value.get('physical_holding_confirmed') is not False):
            self.error = '新版完成回报缺少本次保持到位与夹爪保留证据'

    def complete(self, terminal):
        return bool(terminal and terminal.get('stage') == 'auto_grasp_complete'
                    and self.hold and not self.error)

    def packet(self):
        return dict(selection=deepcopy(self.ready), hold=deepcopy(self.hold),
                    tracking=deepcopy(self.tracking),
                    validation_error=self.error, physical_holding_confirmed=False)
