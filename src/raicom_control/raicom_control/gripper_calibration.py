"""One physical gripper calibration shared by manual pickup and RGB-D tasks."""
import hashlib
import json
import os
from pathlib import Path

DEFAULT_PROFILE = '/home/ubuntu/raicom_runtime/rgbd_pick/calibration.json'


def load_gripper_profile(path=None):
    path = Path(path or os.environ.get('RAICOM_GRASP_CALIBRATION', DEFAULT_PROFILE))
    data = path.read_bytes()
    profile = json.loads(data)
    value = profile.get('gripper', {})
    if profile.get('schema')!='raicom_grasp_calibration/v1' or value.get('verified') is not True or not value.get('evidence'):
        raise ValueError('夹爪开合标定尚未通过现场确认')
    if any(type(value.get(k)) is not int or not 0<=value[k]<=1000 for k in ('open','close')) or value['open']==value['close']:
        raise ValueError('夹爪开合脉冲无效')
    if type(value.get('tolerance')) is not int or not 1<=value['tolerance']<=30:
        raise ValueError('夹爪读回容差无效')
    # Latest explicit workbench command policy. Keep the historical calibration
    # bytes and their provenance; never relabel new command values as measured.
    return dict(value,open=350,close=50,servo_id=10,
                previous_command_values={'open':value['open'],'close':value['close']},
                command_values_source='user_20260923',
                command_values_match_measurement=value['open']==350 and value['close']==50,
                profile_sha256=hashlib.sha256(data).hexdigest(),path=str(path))
