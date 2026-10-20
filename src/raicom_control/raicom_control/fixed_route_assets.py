"""Immutable map/point binding and lossless restoration for fixed routes."""
import copy
import hashlib
import json
import math

ORDER = ['start', 'shibie', 'pick', 'pick_A', 'pick_B']


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                     allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def pose(point):
    p = point['approach']
    values = [p[k] for k in ('x', 'y', 'yaw_rad')]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Invalid formal pose')
    return values


def restore_missing(current, backup, backup_sha, changed_at):
    if current['map'] != backup['map']:
        raise ValueError('Map identity differs')
    if set(backup['points']) != set(ORDER):
        raise ValueError('Backup must contain exactly the five formal points')
    if not set(current['points']).issubset(ORDER) or 'pick_A' not in current['points']:
        raise ValueError('Unexpected current route; keep it for inspection')
    result = copy.deepcopy(current)
    restored = []
    for name in ORDER:
        pose(backup['points'][name])
        if name not in result['points']:
            result['points'][name] = copy.deepcopy(backup['points'][name])
            restored.append(name)
        pose(result['points'][name])
    result['order'] = list(ORDER)
    if result != current:
        result.setdefault('history', []).append({
            'kind': 'authorized_restore_missing_formal_points',
            'changed_at_utc': changed_at, 'backup_sha256': backup_sha,
            'restored_points': restored, 'order_before': current['order'],
            'order_after': list(ORDER), 'preserved_existing_points': list(current['points'])})
    return result


def binding(route, yaml_sha, pgm_sha):
    if set(route['points']) != set(ORDER):
        raise ValueError('Incomplete formal points')
    return {'map_yaml_sha256': yaml_sha, 'map_pgm_sha256': pgm_sha,
            'formal_points_sha256': fingerprint(route['points'])}


def assert_binding(bundle, route, yaml_sha, pgm_sha):
    if bundle['binding'] != binding(route, yaml_sha, pgm_sha):
        raise ValueError('Fixed route invalidated by map/formal-point changes')
