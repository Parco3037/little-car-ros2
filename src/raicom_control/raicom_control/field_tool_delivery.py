"""Offline-testable PICK_READY/A/B task. No ROS imports or hardware access."""
from copy import deepcopy
import hashlib
import json
import math
import uuid

from .delivery_arm import PHASES, validate_pose
from .field_tool_workspace_model import point_identity


def geometry_identity(document):
    value = {'map': document['map'], 'points': {name: point_identity(p)
             for name, p in document['points'].items()}}
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def settings(document):
    value = deepcopy(document.get('workspace', {}).get('delivery', {}))
    value.setdefault('start', 'start')
    value.setdefault('recognition', 'shibie')
    value.setdefault('expanded', False)
    value.setdefault('request_topic', '/raicom/pick/request')
    value.setdefault('result_topic', '/raicom/pick/result')
    value.setdefault('timeout_s', 120.)
    value.setdefault('calibration', {})
    if type(value['expanded']) is not bool:
        raise ValueError('四件 / 八件模式格式错误')
    if (type(value['timeout_s']) not in (int, float) or not math.isfinite(value['timeout_s'])
            or not 5 <= value['timeout_s'] <= 300):
        raise ValueError('夹取等待时间须为 5–300 秒')
    import re
    for key in ('request_topic', 'result_topic'):
        if not isinstance(value[key], str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z_0-9]*/)*[A-Za-z_][A-Za-z_0-9]*', value[key]):
            raise ValueError('夹取话题名称无效')
    if value['request_topic'] == value['result_topic']:
        raise ValueError('请求与返回话题不能相同')
    return value


def validate_program(document, config, destinations):
    names = ['left_a', 'right_a'] + (['left_b', 'right_b'] if config['expanded'] else [])
    required = names + [config['start'], config['recognition']] + list(destinations.values())
    if set(destinations) != {'A', 'B'} or destinations['A'] == destinations['B']:
        raise ValueError('请选择两个不同的园区点位')
    if any(not name or name not in document['points'] for name in required):
        raise ValueError('请先采集 / 选择起点、识别区、夹取点及两个园区点')
    if len(set(required)) != len(required):
        raise ValueError('起点、识别区、夹取点与园区须使用不同点名')
    c = config['calibration']
    if c.get('geometry_identity') != geometry_identity(document):
        raise ValueError('运输 / 放置姿态尚未按当前地图点位标定')
    if c.get('transport_envelope_verified') is not True:
        raise ValueError('请先现场核验运输姿态与车体净空')
    closed = validate_pose(c.get('transport'))['positions']['10']
    if set(c.get('placements', {})) != {'1', '2', '3', '4'}:
        raise ValueError('需要完整记录四套放置动作')
    for program in c['placements'].values():
        if set(program) != set(PHASES):
            raise ValueError('每套动作需要接近、落位、松爪、撤回四步')
        for p in program.values(): validate_pose(p)
        if any(program[p]['positions']['10'] != closed for p in ('approach', 'place')):
            raise ValueError('接近 / 落位期间必须保持夹紧')
        if program['release']['positions']['10'] == closed:
            raise ValueError('松爪姿态必须包含已标定的夹爪变化')
        if program['release']['positions']['10'] != program['withdraw']['positions']['10']:
            raise ValueError('撤回时须保持松爪状态')
    return [name for name in names for _ in range(2)]


class SignalDelivery:
    TERMINAL = {'complete', 'failed', 'cancelled'}

    def __init__(self, document, config, destinations):
        self.schedule = validate_program(document, config, destinations)
        self.config, self.destinations = deepcopy(config), dict(destinations)
        self.task_id = 'delivery-' + uuid.uuid4().hex
        self.stage, self.target = 'navigation', config['start']
        self.leg = 'start'
        self.completed = 0
        self.slots = {'A': 0, 'B': 0}
        self.park = None
        self.reason = ''
        self.mapping = None
        self.deadline = None
        self.wait_started = None
        self.arrivals, self.placements = [], []

    def _navigate(self, target, leg):
        self.stage, self.target, self.leg = 'navigation', target, leg
        self.deadline = None

    def arrived(self, report, now):
        if self.stage != 'navigation': return False
        valid = (report.get('name') == self.target and report.get('nav_status') == 4
                 and report.get('stopped') is True and report.get('accepted') is True)
        for key, limit in [('position_error_m', .02), ('yaw_error_deg', 10.)]:
            n = report.get(key)
            valid = valid and type(n) in (int, float) and math.isfinite(n) and 0 <= n <= limit
        if not valid:
            self.fail('到点后未同时通过 2 cm / 10° 停车验收')
            return False
        self.arrivals.append(deepcopy(report))
        if self.leg == 'start': self._navigate(self.config['recognition'], 'recognition')
        elif self.leg == 'recognition': self.stage, self.deadline = 'recognition', now + 60.
        elif self.leg == 'pick': self.stage = 'pick_ready'
        else: self.stage = 'transport_arrived'
        return True

    def recognized(self, mapping):
        if self.stage != 'recognition': return False
        if set(mapping) != {'1', '2'} or set(mapping.values()) != {'A', 'B'}:
            raise ValueError('识别映射必须包含序列 1/2 与园区 A/B')
        self.mapping = dict(mapping)
        self._navigate(self.schedule[self.completed], 'pick')
        return True

    def pick_ready(self, now):
        if self.stage != 'pick_ready': raise ValueError('未到夹取点')
        self.stage, self.wait_started = 'waiting_ab', now
        self.deadline = now + self.config['timeout_s']
        return 'PICK_READY'

    def receive(self, signal, received_at, now):
        self.tick(now)
        if (self.stage != 'waiting_ab'
                or received_at < self.wait_started or received_at > now): return False
        if signal == 'PICK_FAILED':
            self.fail('同学夹取流程返回失败'); return False
        if signal not in ('A', 'B'): return False
        if self.slots[signal] >= 4:
            self.fail('该园区四个放置位置已用满')
            return False
        self.park, self.stage = signal, 'transport_pose'
        self.deadline = now + 30.
        return True

    def transport_ready(self, physical_readback_ok):
        if self.stage != 'transport_pose': return False
        if physical_readback_ok is not True:
            self.fail('运输姿态读回失败'); return False
        self._navigate(self.destinations[self.park], 'park')
        return True

    def begin_placement(self, now):
        if self.stage != 'transport_arrived': raise ValueError('未到园区')
        self.stage, self.deadline = 'placing', now + 90.
        return deepcopy(self.config['calibration']['placements'][str(self.slots[self.park] + 1)])

    def placement_readback(self, now):
        if self.stage != 'placing': return False
        self.stage, self.deadline = 'confirm_placement', now + 120.
        return True

    def confirm_placement(self, now, physical_success):
        self.tick(now)
        if self.stage != 'confirm_placement': return False
        if physical_success is not True:
            self.fail('实物落点未通过确认'); return False
        self.slots[self.park] += 1
        self.completed += 1
        self.placements.append({'round': self.completed, 'park': self.park,
                                'slot': self.slots[self.park], 'confirmed_at': now})
        self.deadline = None
        if self.completed == len(self.schedule): self.stage = 'complete'
        else: self._navigate(self.schedule[self.completed], 'pick')
        return True

    def tick(self, now):
        if self.deadline is not None and self.stage not in self.TERMINAL and now >= self.deadline:
            self.fail('等待或动作超时：' + self.stage)

    def fail(self, reason):
        self.stage, self.reason, self.deadline = 'failed', reason, None

    def cancel(self, reason='人工取消'):
        self.stage, self.reason, self.deadline = 'cancelled', reason, None

    def status(self):
        return {'task_id': self.task_id, 'stage': self.stage, 'target': self.target,
                'target_park': self.park, 'completed': self.completed, 'total': len(self.schedule),
                'slots': dict(self.slots), 'mapping': self.mapping, 'reason': self.reason,
                'pick_point': self.schedule[min(self.completed, len(self.schedule)-1)],
                'slot': self.slots[self.park]+1 if self.park and self.slots[self.park]<4 else None}
