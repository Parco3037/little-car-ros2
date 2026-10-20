"""Optional field UI settings; original poses and navigation policy stay intact."""
from copy import deepcopy
import hashlib
import json
import math

from .field_tool_model import POINT_NAME_PATTERN, utc_now
from .delivery_arm import validate_pose
from .field_tool_recovery import DEFAULT_MAX_RECOVERIES, recovery_limit


def point_identity(point):
    return hashlib.sha256(json.dumps(
        {key: point[key] for key in ('reference', 'approach')},
        sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def workspace_settings(document, *, require_current_pose=False):
    value = deepcopy(document.get('workspace', {}))
    if not isinstance(value, dict):
        raise ValueError('高级编排设置格式错误')
    value.setdefault('delay_s', 0.)
    value.setdefault('destinations', {'A': '', 'B': ''})
    value.setdefault('points', {})
    value.setdefault('max_recoveries', DEFAULT_MAX_RECOVERIES)
    recovery_limit(value['max_recoveries'])
    if 'speeds' in value:
        from .field_tool_speed import speed_settings
        speed_settings(value['speeds'])
    if (not isinstance(value['destinations'], dict) or not isinstance(value['points'], dict)
            or any(not isinstance(p, dict) for p in value['points'].values())):
        raise ValueError('高级点位 / 目的地格式错误')
    for number in [value['delay_s']] + [p.get('dwell_s', 0.) for p in value['points'].values()]:
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or not 0 <= number <= 3600:
            raise ValueError('延时 / 停留时间须为 0–3600 秒')
    if set(value['destinations']) != {'A', 'B'}:
        raise ValueError('目的地必须包含 A 和 B')
    if any(not isinstance(v, str) for v in value['destinations'].values()):
        raise ValueError('目的地必须为点名')
    for name, settings in value['points'].items():
        if name not in document['points']:
            continue  # Deleted points remain in history; never dispatch them.
        if not isinstance(settings.get('note', ''), str) or len(settings.get('note', '')) > 200:
            raise ValueError('备注最多 200 字')
        if type(settings.get('apply_arm_pose', False)) is not bool:
            raise ValueError('姿态执行开关必须明确为布尔值')
        if settings.get('arm_pose') is not None:
            validate_pose(settings['arm_pose'])
        if settings.get('apply_arm_pose'):
            if require_current_pose and settings.get('pose_fingerprint') != point_identity(document['points'][name]):
                raise ValueError(f'{name} 坐标已变化，请重新记录机械臂姿态')
            validate_pose(settings.get('arm_pose'))
    return value


def execution_identity(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def save_settings(document, settings):
    candidate = deepcopy(document)
    candidate['workspace'] = deepcopy(settings)
    workspace_settings(candidate)
    candidate.setdefault('history', []).append({
        'at_utc': utc_now(), 'kind': 'workspace_settings',
        'before': deepcopy(document.get('workspace')), 'after': deepcopy(settings)})
    return candidate


def duplicate_point(document, source, name):
    if source not in document['points'] or not POINT_NAME_PATTERN.fullmatch(name):
        raise ValueError('新点名须为 1–64 位字母、数字或下划线')
    if name in document['points']:
        raise ValueError('点名已存在，请使用新名称')
    candidate = deepcopy(document)
    point = deepcopy(candidate['points'][source])
    point['name'] = name
    candidate['points'][name] = point
    candidate['order'].insert(candidate['order'].index(source) + 1, name)
    # Pose replay is deliberately opt-in for the new point.
    settings = workspace_settings(candidate)
    if source in settings['points']:
        settings['points'][name] = deepcopy(settings['points'][source])
        settings['points'][name]['apply_arm_pose'] = False
    candidate['workspace'] = settings
    candidate['history'].append({'at_utc': utc_now(), 'kind': 'duplicate_point',
                                 'source': source, 'name': name, 'after': deepcopy(point)})
    return candidate
