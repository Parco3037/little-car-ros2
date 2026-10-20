"""Pure data model for the RAICOM field mapping and navigation tool.

This module owns route files and map bindings only. It has no ROS, actuator,
permission, subprocess, or user-interface side effects.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile

import yaml

from .map_preflight import validate_saved_map
from .route_points_logic import (
    combine_map_binding,
    loaded_map_binding,
    saved_map_binding,
)


SCHEMA = 'raicom_field_route/v2'
POINT_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_]{1,64}$')
LABEL_MAX = 80
DEFAULT_POSITION_TOLERANCE_M = 0.02
DEFAULT_YAW_TOLERANCE_DEG = 10.0
LEGACY_ROUTE_SCHEMA = 'raicom_manual_route/v1'
LEGACY_APPROACH_SCHEMA = 'raicom_navigation_approach/v2'


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def wrap_yaw(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('yaw must be finite')
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def validate_pose(x: float, y: float, yaw: float) -> dict:
    values = (float(x), float(y), float(yaw))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pose contains a non-finite value')
    if not -100.0 <= values[0] <= 100.0 or not -100.0 <= values[1] <= 100.0:
        raise ValueError('pose is outside the 100 m field guard')
    wrapped = wrap_yaw(values[2])
    return {
        'x': values[0],
        'y': values[1],
        'yaw_rad': wrapped,
        'yaw_deg': math.degrees(wrapped),
    }


def _validate_point_name(name: str) -> str:
    if not isinstance(name, str) or not POINT_NAME_PATTERN.fullmatch(name):
        raise ValueError('point name must be 1-64 letters, numbers, or underscores')
    return name


def _validate_label(label: str) -> str:
    label = str(label).strip()
    if not label or len(label) > LABEL_MAX:
        raise ValueError('point label must be 1-80 non-whitespace characters')
    return label


def _validate_pose_document(pose: dict, *, require_approach: bool) -> dict:
    if not isinstance(pose, dict):
        raise ValueError('pose must be an object')
    try:
        values = validate_pose(pose['x'], pose['y'], pose['yaw_rad'])
    except KeyError as exc:
        raise ValueError('pose is missing x, y, or yaw_rad') from exc
    if not math.isclose(
        float(pose.get('yaw_deg', math.degrees(values['yaw_rad']))),
        values['yaw_deg'],
        abs_tol=1e-6,
    ):
        raise ValueError('pose yaw units disagree')
    if require_approach:
        offset = pose.get('offset_from_reference_m')
        if (not isinstance(offset, list) or len(offset) != 2
                or not all(isinstance(value, (int, float)) for value in offset)
                or not all(math.isfinite(float(value)) for value in offset)
                or math.hypot(float(offset[0]), float(offset[1])) > 0.50):
            raise ValueError('approach offset must be a finite 2D offset within 0.50 m')
    return {
        **values,
        **({'offset_from_reference_m': [float(v) for v in offset]} if require_approach else {}),
        'recorded_at_utc': str(pose.get('recorded_at_utc') or utc_now()),
        'source': str(pose.get('source') or 'manual field survey'),
    }


def _validate_point(point: dict) -> dict:
    if not isinstance(point, dict):
        raise ValueError('point must be an object')
    name = _validate_point_name(str(point.get('name', '')))
    label = _validate_label(point.get('label', name))
    reference = _validate_pose_document(
        point.get('reference') or {}, require_approach=False)
    approach = point.get('approach')
    if approach is None:
        approach = {
            'x': reference['x'],
            'y': reference['y'],
            'yaw_rad': reference['yaw_rad'],
            'yaw_deg': reference['yaw_deg'],
            'offset_from_reference_m': [0.0, 0.0],
            'recorded_at_utc': reference['recorded_at_utc'],
            'source': reference['source'],
        }
    approach = _validate_pose_document(approach, require_approach=True)
    try:
        position_tolerance = float(point.get(
            'position_tolerance_m', DEFAULT_POSITION_TOLERANCE_M))
        yaw_tolerance_deg = float(point.get(
            'yaw_tolerance_deg', DEFAULT_YAW_TOLERANCE_DEG))
    except (TypeError, ValueError) as exc:
        raise ValueError('point tolerances are invalid') from exc
    if not 0.01 <= position_tolerance <= 0.20:
        raise ValueError('position tolerance must be 0.01..0.20 m')
    if not 1.0 <= yaw_tolerance_deg <= 30.0:
        raise ValueError('yaw tolerance must be 1..30 degrees')
    return {
        'name': name,
        'label': label,
        'reference': reference,
        'approach': approach,
        'position_tolerance_m': position_tolerance,
        'yaw_tolerance_deg': yaw_tolerance_deg,
    }


def _saved_identity(map_file) -> dict:
    return saved_map_binding(validate_saved_map(map_file))


def _loaded_identity(message) -> dict | None:
    return loaded_map_binding(message) if message is not None else None


def binding_for_map(map_file, loaded_message=None) -> dict:
    saved = _saved_identity(map_file)
    loaded = _loaded_identity(loaded_message)
    return combine_map_binding(saved, loaded) if loaded is not None else {'saved': saved}


def _validate_binding(document: dict, binding: dict) -> None:
    stored = document.get('map')
    if not isinstance(stored, dict):
        raise ValueError('route map binding is missing')
    expected_saved = binding.get('saved', stored.get('saved'))
    if stored.get('saved') != expected_saved:
        raise ValueError('route belongs to a different saved map')
    if binding.get('loaded') is not None:
        if stored.get('loaded') != binding['loaded']:
            raise ValueError('route does not match the currently loaded map')


def new_route(map_file, loaded_message=None) -> dict:
    validate_saved_map(map_file)
    return {
        'schema': SCHEMA,
        'frame': 'map',
        'map': binding_for_map(map_file, loaded_message),
        'order': [],
        'points': {},
        'history': [],
    }


def set_start_localization(document: dict, pose: dict) -> None:
    """Persist an initialization hint, not an accepted navigation waypoint."""
    document['start_localization'] = {
        'mode': 'fixed_start', 'frame': 'map', 'name': 'start',
        'label': '中转区', 'requires_same_position_and_heading': True,
        'pose': {**validate_pose(pose['x'], pose['y'], pose['yaw_rad']),
                 'recorded_at_utc': utc_now(), 'source': 'stationary mapping start'},
    }


def start_localization_pose(document: dict) -> dict | None:
    hint = document.get('start_localization')
    if hint is None:
        return None  # Existing maps do not silently opt in.
    if (not isinstance(hint, dict) or hint.get('mode') != 'fixed_start'
            or hint.get('frame') != 'map' or hint.get('name') != 'start'
            or hint.get('requires_same_position_and_heading') is not True):
        raise ValueError('invalid fixed-start localization hint')
    stored = _validate_pose_document(hint.get('pose'), require_approach=False)
    # A later fixed-map reference survey supersedes the original SLAM hint;
    # an approach offset must never move the physical startup anchor.
    point = document.get('points', {}).get('start')
    return _validate_point(point)['reference'] if point is not None else stored


def read_route_text(text: str, map_file, loaded_message=None) -> dict:
    binding = binding_for_map(map_file, loaded_message)
    if text is None or not text.strip():
        return new_route(map_file, loaded_message)
    try:
        document = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError('route file is not valid JSON') from exc
    if not isinstance(document, dict) or document.get('schema') != SCHEMA:
        raise ValueError('unsupported route schema')
    if document.get('frame') != 'map':
        raise ValueError('route must use the map frame')
    _validate_binding(document, binding)
    points = document.get('points')
    if not isinstance(points, dict):
        raise ValueError('route points are missing')
    order = document.get('order')
    if not isinstance(order, list) or any(
            not isinstance(item, str) for item in order):
        raise ValueError('route order must be a list of point names')
    if set(order) != set(points):
        raise ValueError('route order must contain every point exactly once')
    if len(order) != len(set(order)):
        raise ValueError('route order contains duplicates')
    history = document.get('history')
    if not isinstance(history, list):
        raise ValueError('route history is missing')
    document['points'] = {
        _validate_point_name(name): _validate_point(point)
        for name, point in points.items()
    }
    if any(name != point['name'] for name, point in document['points'].items()):
        raise ValueError('point name differs from route key')
    document['order'] = [_validate_point_name(name) for name in order]
    start_localization_pose(document)
    return document


def read_route_file(path, map_file, loaded_message=None) -> dict:
    route_path = Path(path)
    if not route_path.exists():
        return new_route(map_file, loaded_message)
    return read_route_text(route_path.read_text(encoding='utf-8'), map_file, loaded_message)


def legacy_route_candidates(map_file, roots) -> list[Path]:
    map_path = Path(validate_saved_map(map_file))
    candidates = []
    for root in roots:
        route = Path(root) / f'route_{map_path.stem}.json'
        if route.is_file():
            candidates.append(route)
    return candidates


def migrate_legacy_route(map_file, roots) -> dict | None:
    """Import the read-only v1 five-station journal into the v2 field model."""

    map_path = Path(validate_saved_map(map_file))
    route_candidates = legacy_route_candidates(map_path, roots)
    if not route_candidates:
        return None
    route_path = route_candidates[0]
    raw = route_path.read_bytes()
    try:
        document = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError('legacy route is not valid JSON') from exc
    if document.get('schema') != LEGACY_ROUTE_SCHEMA:
        raise ValueError('legacy route schema is unsupported')
    binding = binding_for_map(map_path)
    stored_saved = document.get('map', {}).get('saved')
    if stored_saved != binding['saved']:
        raise ValueError('legacy route belongs to a different map')
    migrated = new_route(map_path)
    for name, point in document.get('points', {}).items():
        if not isinstance(point, dict):
            raise ValueError('legacy route point is invalid')
        record_point(
            migrated,
            name,
            point.get('label', name),
            point['x'],
            point['y'],
            point['yaw_rad'],
            timestamp=str(point.get('recorded_at_utc') or utc_now()),
        )
    migrated['legacy_source'] = {
        'route_file': str(route_path),
        'route_sha256': hashlib.sha256(raw).hexdigest(),
    }

    approach_path = None
    for root in roots:
        candidate = (
            Path(root)
            / f'navigation_approach_{map_path.stem}.json'
        )
        if candidate.is_file():
            approach_path = candidate
            break
    if approach_path is not None:
        raw_approach = approach_path.read_bytes()
        try:
            approach = json.loads(raw_approach)
        except json.JSONDecodeError as exc:
            raise ValueError('legacy approach file is not valid JSON') from exc
        if approach.get('schema') != LEGACY_APPROACH_SCHEMA:
            raise ValueError('legacy approach schema is unsupported')
        if approach.get('route_sha256') != hashlib.sha256(raw).hexdigest():
            raise ValueError('legacy approach is not bound to the route')
        if approach.get('map_yaml_sha256') != binding['saved']['yaml_sha256']:
            raise ValueError('legacy approach is not bound to the map YAML')
        points = approach.get('points', {})
        if set(points) != set(migrated['points']):
            raise ValueError('legacy approach point set differs from route')
        for name, pose in points.items():
            offset = pose.get('offset_from_work_pose_m')
            if not isinstance(offset, list) or len(offset) != 2:
                raise ValueError('legacy approach offset is invalid')
            set_approach_offset(migrated, name, offset[0], offset[1])
        migrated['legacy_source']['approach_file'] = str(approach_path)
        migrated['legacy_source']['approach_sha256'] = (
            hashlib.sha256(raw_approach).hexdigest())
    return migrated


def route_file_for_map(map_file, routes_dir) -> Path:
    map_path = Path(validate_saved_map(map_file))
    routes_path = Path(routes_dir)
    if not routes_path.is_absolute() or not map_path.is_absolute():
        raise ValueError('map and route directories must be absolute')
    legacy = routes_path / (map_path.stem + '.route.json')
    identity = _saved_identity(map_path)
    if legacy.exists():
        existing = json.loads(legacy.read_text(encoding='utf-8'))
        if existing.get('map', {}).get('saved') == identity:
            return legacy
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:16]
    return routes_path / f'{map_path.stem}.{digest}.route.json' 


def _offset_in_world(reference: dict, dx: float, dy: float) -> dict:
    yaw = float(reference['yaw_rad'])
    x = float(reference['x']) + dx * math.cos(yaw) - dy * math.sin(yaw)
    y = float(reference['y']) + dx * math.sin(yaw) + dy * math.cos(yaw)
    return validate_pose(x, y, yaw)


def set_approach_offset(document: dict, name: str, dx: float, dy: float) -> dict:
    name = _validate_point_name(name)
    if name not in document['points']:
        raise ValueError('unknown point')
    try:
        dx, dy = float(dx), float(dy)
    except (TypeError, ValueError) as exc:
        raise ValueError('approach offset is invalid') from exc
    if not math.isfinite(dx) or not math.isfinite(dy) or math.hypot(dx, dy) > 0.50:
        raise ValueError('approach offset must be within 0.50 m')
    point = document['points'][name]
    before = deepcopy(point['approach'])
    world = _offset_in_world(point['reference'], dx, dy)
    after = {
        **world,
        'offset_from_reference_m': [dx, dy],
        'recorded_at_utc': utc_now(),
        'source': 'approach offset derived from reference pose',
    }
    point['approach'] = after
    _append_history(document, name, 'approach', before, after)
    return document


def record_point(
        document: dict,
        name: str,
        label: str,
        x: float,
        y: float,
        yaw: float,
        *,
        kind: str = 'reference',
        timestamp: str | None = None,
) -> dict:
    name = _validate_point_name(name)
    label = _validate_label(label)
    if kind not in {'reference', 'approach'}:
        raise ValueError('point kind must be reference or approach')
    pose = validate_pose(x, y, yaw)
    stamp = timestamp or utc_now()
    point = document['points'].get(name)
    if point is None:
        reference = {
            **pose,
            'recorded_at_utc': stamp,
            'source': 'reference survey pose',
        }
        approach = {
            **pose,
            'offset_from_reference_m': [0.0, 0.0],
            'recorded_at_utc': stamp,
            'source': 'approach pose defaults to reference',
        }
        document['points'][name] = {
            'name': name,
            'label': label,
            'reference': reference,
            'approach': approach,
            'position_tolerance_m': DEFAULT_POSITION_TOLERANCE_M,
            'yaw_tolerance_deg': DEFAULT_YAW_TOLERANCE_DEG,
        }
        document['order'].append(name)
        return document

    before = deepcopy(point)
    if kind == 'reference':
        delta_yaw = wrap_yaw(pose['yaw_rad'] - point['reference']['yaw_rad'])
        point['reference'] = {
            **pose,
            'recorded_at_utc': stamp,
            'source': 'reference survey correction',
        }
        point['label'] = label
        old_offset = point['approach']['offset_from_reference_m']
        approach_world = _offset_in_world(
            point['reference'], old_offset[0], old_offset[1])
        # Preserve the operator-chosen approach offset while moving it with
        # the corrected reference, and preserve its relative heading too.
        approach_yaw = wrap_yaw(point['approach']['yaw_rad'] + delta_yaw)
        point['approach'] = {
            **validate_pose(
                approach_world['x'],
                approach_world['y'],
                approach_yaw,
            ),
            'offset_from_reference_m': deepcopy(old_offset),
            'recorded_at_utc': stamp,
            'source': 'approach pose moved with corrected reference',
        }
    else:
        reference = point['reference']
        dx, dy = pose['x'] - reference['x'], pose['y'] - reference['y']
        c, sin_yaw = math.cos(reference['yaw_rad']), math.sin(reference['yaw_rad'])
        offset = [c * dx + sin_yaw * dy, -sin_yaw * dx + c * dy]
        if math.hypot(*offset) > 0.50:
            raise ValueError('approach offset must be within 0.50 m')
        point['approach'] = {
            **pose,
            'offset_from_reference_m': offset,
            'recorded_at_utc': stamp,
            'source': 'approach survey correction',
        }
    _append_history(document, name, kind, before, deepcopy(point))
    return document


def set_point_label(document: dict, name: str, label: str) -> dict:
    name = _validate_point_name(name)
    label = _validate_label(label)
    if name not in document['points']:
        raise ValueError('unknown point')
    before = deepcopy(document['points'][name])
    document['points'][name]['label'] = label
    _append_history(document, name, 'label', before, deepcopy(document['points'][name]))
    return document


def set_point_tolerances(
        document: dict, name: str, position_m: float, yaw_deg: float) -> dict:
    name = _validate_point_name(name)
    if name not in document['points']:
        raise ValueError('unknown point')
    before = deepcopy(document['points'][name])
    point = deepcopy(document['points'][name])
    point['position_tolerance_m'] = float(position_m)
    point['yaw_tolerance_deg'] = float(yaw_deg)
    _validate_point(point)
    document['points'][name] = point
    _append_history(document, name, 'tolerances', before, deepcopy(point))
    return document


def delete_point(document: dict, name: str) -> dict:
    name = _validate_point_name(name)
    if name not in document['points']:
        raise ValueError('unknown point')
    before = deepcopy(document['points'][name])
    del document['points'][name]
    document['order'] = [item for item in document['order'] if item != name]
    document['history'].append({
        'point': name,
        'kind': 'delete',
        'before': before,
        'after': None,
        'changed_at_utc': utc_now(),
    })
    return document


def set_order(document: dict, order: list[str]) -> dict:
    order = [_validate_point_name(str(name)) for name in order]
    if set(order) != set(document['points']) or len(order) != len(set(order)):
        raise ValueError('order must contain every point exactly once')
    if order == document['order']:
        return document
    before = deepcopy(document['order'])
    document['order'] = order
    document['history'].append({
        'point': None,
        'kind': 'order',
        'before': before,
        'after': deepcopy(order),
        'changed_at_utc': utc_now(),
    })
    return document


def _append_history(document, name, kind, before, after) -> None:
    document.setdefault('history', []).append({
        'point': name,
        'kind': kind,
        'before': before,
        'after': after,
        'changed_at_utc': utc_now(),
    })


def navigation_pose(point: dict) -> dict:
    return deepcopy(point.get('approach', point['reference']))


def atomic_write_route(path, document: dict) -> None:
    text = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    target = Path(path)
    if document.get('schema') != SCHEMA or document.get('frame') != 'map':
        raise ValueError('unsupported route schema or frame')
    points, order = document.get('points', {}), document.get('order', [])
    if len(order) != len(set(order)) or set(order) != set(points):
        raise ValueError('route order must contain every point exactly once')
    for name, point in points.items():
        if _validate_point(point)['name'] != name:
            raise ValueError('point name differs from route key')
    if target.exists():
        old = json.loads(target.read_text(encoding='utf-8'))
        if old.get('map', {}).get('saved') != document.get('map', {}).get('saved'):
            raise ValueError('refusing to overwrite route for a different saved map')
    target.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(
        prefix='.' + target.name + '.', suffix='.tmp', dir=target.parent)
    try:
        with os.fdopen(handle, 'w', encoding='utf-8', newline='\n') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def route_digest(document: dict) -> str:
    payload = json.dumps(
        {
            'schema': document['schema'],
            'frame': document['frame'],
            'map': document['map'],
            'order': document['order'],
            'points': document['points'],
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


def discover_maps(root) -> list[Path]:
    maps_dir = Path(root)
    if not maps_dir.is_dir():
        return []
    results = []
    for path in maps_dir.glob('*.yaml'):
        if path.name.startswith('.'):
            continue
        try:
            validate_saved_map(path)
        except (OSError, ValueError, yaml.YAMLError):
            continue
        results.append(path.resolve())
    return sorted(results)


def publish_staged_map(staged_yaml, target_dir) -> Path:
    """Publish a validated map_saver pair without replacing an existing pair."""

    staged = Path(staged_yaml).resolve(strict=True)
    target = Path(target_dir).resolve()
    validate_saved_map(staged)
    metadata = yaml.safe_load(staged.read_text(encoding='utf-8'))
    staged_image = (staged.parent / metadata['image']).resolve(strict=True)
    if metadata['image'] != staged_image.name:
        raise ValueError('staged map image must be a sibling filename')
    final_yaml = target / staged.name
    final_image = target / staged_image.name
    for path in (final_yaml, final_image):
        if os.path.lexists(path):
            raise FileExistsError(f'refusing to overwrite map file: {path}')
    target.mkdir(parents=True, exist_ok=True)
    os.link(staged_image, final_image)
    try:
        os.link(staged, final_yaml)
    except Exception:
        try:
            os.unlink(final_image)
        except OSError:
            pass
        raise
    return final_yaml


def next_map_name(root, label: str) -> str:
    label = str(label).strip().lower()
    safe = re.sub(r'[^a-z0-9_-]+', '-', label).strip('-_')
    if not safe:
        safe = 'field'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    stem = f'{stamp}_{safe}'
    maps_dir = Path(root)
    maps_dir.mkdir(parents=True, exist_ok=True)
    if not any((maps_dir / f'{stem}{ext}').exists() for ext in ('.yaml', '.pgm')):
        return stem
    for index in range(1, 1000):
        candidate = f'{stem}_{index}'
        if not any((maps_dir / f'{candidate}{ext}').exists() for ext in ('.yaml', '.pgm')):
            return candidate
    raise FileExistsError('could not allocate a unique map filename')
