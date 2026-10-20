"""Load field measurements for the competition backend; never invent a pose."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from .delivery_arm import PHASES, validate_pose
from .field_tool_delivery import geometry_identity
from .field_tool_workspace_model import point_identity

SCHEMA = 'raicom.competition.calibration/v1'


def number(value, lower, upper):
    return type(value) in (int, float) and math.isfinite(value) and lower <= value <= upper


class Calibration:
    """An immutable snapshot. Evidence paths must stay inside its own directory."""
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.root = self.path.parent
        data = self.path.read_bytes()
        self.sha256 = hashlib.sha256(data).hexdigest()
        self.value = json.loads(data)
        c = self.value
        if c.get('schema') != SCHEMA:
            raise ValueError('比赛动作标定文件格式错误')
        if c.get('geometry_identity') != geometry_identity(c.get('route', {})):
            raise ValueError('标定与地图/点位指纹不一致')
        self.dependencies = {self.path: self.sha256}
        self.evidence(c['gripper']['evidence'])
        g = c['gripper']
        if (any(type(g.get(k)) is not int for k in ('open', 'close', 'holding_min', 'holding_max', 'tolerance'))
                or not 0 <= g['close'] < g['holding_min'] <= g['holding_max'] < g['open'] <= 1000
                or not 1 <= g['tolerance'] <= 15
                or g['holding_min'] - g['tolerance'] <= g['close']
                or g['holding_max'] + g['tolerance'] >= g['open']):
            raise ValueError('缺少可区分空夹、夹稳和张开的实测夹爪范围')
        wrist = c['fixed_wrist']
        self.evidence(wrist['evidence'])
        if (type(wrist.get('position')) is not int or not 0 <= wrist['position'] <= 1000
                or type(wrist.get('tolerance')) is not int or not 1 <= wrist['tolerance'] <= 25):
            raise ValueError('固定腕部读回范围无效')
        classification = c['classification']
        self.evidence(classification['evidence'])
        self.pose(classification['pose'])
        size, roi = classification.get('image_size'), classification.get('roi')
        if (classification.get('verified') is not True or not isinstance(size, list) or len(size) != 2
                or any(type(n) is not int or n <= 0 for n in size)
                or not isinstance(roi, list) or len(roi) != 4 or any(not number(n, 0, max(size)) for n in roi)
                or not 0 <= roi[0] < roi[2] <= size[0] or not 0 <= roi[1] < roi[3] <= size[1]):
            raise ValueError('夹持视野尚未实测：分辨率、ROI和观察姿态不可猜测')
        self.catalog = dict(geometry_identity=c['geometry_identity'], transport={}, placements={})
        for name, profile in c.get('transport', {}).items():
            if profile.get('verified') is not True or profile.get('holding_preserved') is not True:
                raise ValueError('运输姿态尚未通过持物与空间包络验证')
            self.evidence(profile['evidence'])
            self.pose(profile['entry_pose']); self.pose(profile['pose'])
            self.catalog['transport'][name] = dict(verified=True, holding_preserved=True, evidence=profile['evidence'])
        for name, profile in c.get('placements', {}).items():
            if profile.get('verified') is not True or set(profile.get('program', {})) != set(PHASES):
                raise ValueError('放置动作缺少四个步骤或实物验证')
            self.evidence(profile['evidence']); self.pose(profile['entry_pose'])
            for pose in profile['program'].values(): self.pose(pose)
            program = profile['program']
            # Release opens only finger 10; it must not hide an arm movement.
            if any(program['release']['positions'][k] != program['place']['positions'][k] for k in ('1','2','3','4','5')):
                raise ValueError('松爪步骤不得同时改变机械臂位置')
            if any(abs(program[p]['positions']['10'] - g['open']) > g['tolerance'] for p in ('release','withdraw')):
                raise ValueError('松爪/撤回须使用实测张开值')
            points = profile.get('landings', {})
            if not points: raise ValueError('动作尚无对应园区落点验证')
            fingerprints = []
            for point, proof in points.items():
                self.evidence(proof['evidence'])
                if proof.get('fingerprint') != point_identity(c['route']['points'][point]):
                    raise ValueError('园区坐标已改变，需要重新核验落位')
                fingerprints.append(proof['fingerprint'])
            self.catalog['placements'][name] = dict(verified=True, phases=list(PHASES),
                evidence=profile['evidence'], verified_point_fingerprints=fingerprints)
        if not self.catalog['transport'] or len(self.catalog['placements']) < 4:
            raise ValueError('请先采集运输姿态和四套放置动作')
        feedback = c.get('placement_feedback', {})
        if feedback.get('verified') is True:
            if not isinstance(feedback.get('provider_id'),str) or not feedback['provider_id']:
                raise ValueError('实物检测程序编号无效')
            self.evidence(feedback['evidence'])
        elif feedback.get('verified') is not False:
            raise ValueError('须明确记录实物检测是否已经验证')

    def evidence(self, reference):
        if (not isinstance(reference, dict) or not isinstance(reference.get('path'), str)
                or not reference['path'] or not isinstance(reference.get('sha256'), str)):
            raise ValueError('缺少可核验的标定证据文件')
        relative = Path(reference['path'])
        path = (self.root / relative).resolve()
        if relative.is_absolute() or self.root not in path.parents:
            raise ValueError('标定证据路径必须在标定目录内')
        data = path.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if digest != reference['sha256']: raise ValueError('标定证据内容已改变')
        self.dependencies[path] = digest
        return data

    def pose(self, pose):
        validate_pose(pose)
        evidence = json.loads(self.evidence(pose['measurement']))
        reads = evidence.get('physical_reads')
        if (evidence.get('physical_readback_verified') is not True or evidence.get('command_sent') is not False
                or not evidence.get('boot_id') or not number(evidence.get('recorded_at_unix'), 1, 1e12)
                or not isinstance(reads, list) or len(reads) != 2):
            raise ValueError('姿态必须有两次真实舵机读回采集')
        for read in reads:
            if (not isinstance(read, dict) or set(read) != set(pose['positions'])
                    or any(type(v) is not int or not 0 <= v <= 1000 for v in read.values())):
                raise ValueError('姿态采集的舵机读回不完整')
        if reads[-1] != pose['positions'] or any(abs(reads[0][k]-reads[1][k]) > 3 for k in reads[0]):
            raise ValueError('姿态脉冲与原始采集证据不一致或采集不稳定')
        wrist = self.value['fixed_wrist']
        if abs(pose['positions']['5'] - wrist['position']) > wrist['tolerance']:
            raise ValueError('比赛动作不得改变固定腕部5号舵机')
        return pose

    def unchanged(self):
        return all(path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest
                   for path, digest in self.dependencies.items())

    def program(self, request):
        if request['geometry_identity'] != self.value['geometry_identity']:
            raise ValueError('请求地图与标定不一致')
        if request['operation'] == 'transport':
            if request.get('keep_gripper_closed') is not True:
                raise ValueError('运输必须明确保持夹紧')
            p = self.value['transport'][request['profile_id']]
            return deepcopy(p['entry_pose']), [('transport', deepcopy(p['pose']), 'holding')]
        p = self.value['placements'][request['profile_id']]
        if (request.get('park') not in ('A','B') or type(request.get('slot')) is not int
                or not 1 <= request['slot'] <= 4 or request.get('point') not in p['landings']):
            raise ValueError('请求落点尚未标定')
        return deepcopy(p['entry_pose']), [(phase, deepcopy(p['program'][phase]),
            'holding' if phase in ('approach','place') else 'release' if phase == 'release' else 'open') for phase in PHASES]


def classify_frame(payload, profile):
    from .vision_decision import decide_markers
    decision=decide_markers(payload,roi=profile.get('roi'),expected_size=profile.get('image_size'))
    if decision['label'] is None:raise ValueError(decision['reason'])
    return decision['label']
