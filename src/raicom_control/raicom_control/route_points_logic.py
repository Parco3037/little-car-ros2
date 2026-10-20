"""Five named manual-survey stations, bound to the saved and loaded map.

This journal is data, not a mission configuration. It cannot enable navigation
or actuators. Re-recording keeps the previous measurement in history.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import yaml

from .map_preflight import validate_saved_map
from .scene_pose_recorder_logic import ScenePose, validate_pose


STATIONS = {
    'start': '启动区', 'recognition': '识别区', 'pickup': '夹取区',
    'park_1': '园区一', 'park_2': '园区二',
}
SCHEMA = 'raicom_manual_route/v1'


def saved_map_binding(map_file):
    path = Path(validate_saved_map(map_file))
    raw = path.read_bytes()
    metadata = yaml.safe_load(raw)
    image = (path.parent / metadata['image']).resolve(strict=True)
    return {
        'yaml_sha256': hashlib.sha256(raw).hexdigest(),
        'image_sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
        'resolution': float(metadata['resolution']),
        'origin': [float(v) for v in metadata['origin']],
    }


def loaded_map_binding(message):
    info = message.info
    if len(message.data) != info.width * info.height or any(v < -1 or v > 100 for v in message.data):
        raise ValueError('Loaded map has invalid occupancy data')
    q = info.origin.orientation
    if abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
        raise ValueError('Map origin must be planar')
    from .scene_pose_recorder_logic import quaternion_to_yaw
    return {
        'frame': message.header.frame_id,
        'width': int(info.width), 'height': int(info.height),
        'resolution': float(info.resolution),
        'origin': [float(info.origin.position.x), float(info.origin.position.y),
                   quaternion_to_yaw(q.x, q.y, q.z, q.w)],
        'cells_sha256': hashlib.sha256(bytes(v + 1 for v in message.data)).hexdigest(),
    }


def combine_map_binding(saved, loaded):
    if loaded['frame'] != 'map' or loaded['width'] <= 0 or loaded['height'] <= 0:
        raise ValueError('Loaded map is missing or invalid')
    if not math.isclose(saved['resolution'], loaded['resolution'], rel_tol=1e-6):
        raise ValueError('Saved and loaded map resolutions differ')
    if any(not math.isclose(a, b, abs_tol=1e-6) for a, b in zip(saved['origin'], loaded['origin'])):
        raise ValueError('Saved and loaded map origins differ')
    return {'saved': deepcopy(saved), 'loaded': deepcopy(loaded)}


def read_route(text, binding):
    if text is None:
        return {'schema': SCHEMA, 'frame': 'map', 'map': deepcopy(binding),
                'points': {}, 'history': []}
    document = json.loads(text)
    if not isinstance(document, dict) or document.get('schema') != SCHEMA:
        raise ValueError('Unsupported route journal; original file preserved')
    if document.get('frame') != 'map' or document.get('map') != binding:
        raise ValueError('Route belongs to a different map; do not reuse its points')
    points = document.get('points')
    if not isinstance(points, dict) or set(points) - set(STATIONS):
        raise ValueError('Invalid route stations')
    if not isinstance(document.get('history'), list):
        raise ValueError('Invalid route history')
    for point in points.values():
        validate_pose(ScenePose(float(point['x']), float(point['y']), float(point['yaw_rad'])))
        if not math.isclose(float(point['yaw_deg']), math.degrees(float(point['yaw_rad'])), abs_tol=1e-6):
            raise ValueError('Route yaw units disagree')
    return document


def record_route(text, station, pose, binding, timestamp=None):
    if station not in STATIONS:
        raise ValueError('Unknown route station')
    validate_pose(pose)
    document = read_route(text, binding)
    # Archive the prior point instead of silently destroying it on re-record.
    if station in document['points']:
        document['history'].append({'station': station, 'point': deepcopy(document['points'][station])})
    document['points'][station] = {
        'label': STATIONS[station], 'x': float(pose.x), 'y': float(pose.y),
        'yaw_rad': float(pose.yaw), 'yaw_deg': math.degrees(pose.yaw),
        'recorded_at_utc': timestamp or datetime.now(timezone.utc).isoformat(),
        'source': 'map->base_footprint; guarded zero-speed hold; operator-selected station',
    }
    return json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n'


def route_summary(document):
    lines = [f"{len(document['points'])}/5 recorded; this journal does not enable navigation"]
    for station, label in STATIONS.items():
        point = document['points'].get(station)
        lines.append(f'{station} / {label}: ' + (
            f"x={point['x']:.3f}, y={point['y']:.3f}, yaw={point['yaw_deg']:.1f} deg"
            if point else 'NOT RECORDED'))
    return '\n'.join(lines)
