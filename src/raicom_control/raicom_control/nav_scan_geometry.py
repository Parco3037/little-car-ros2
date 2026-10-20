"""Navigation-only rejection of body returns within the requested 5 mm margin.

No isolated-point or range-radius deletion: outside points, unknown beams and
the scan geometry remain unchanged. A rejected return becomes unknown (NaN).
"""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml

NAV_SCAN_TOPIC = '/raicom/navigation_scan'
BODY_MASK_MARGIN_M = 0.005


def footprint_points(value):
    points = json.loads(value) if isinstance(value, str) else value
    if len(points) < 3 or any(len(p) != 2 for p in points):
        raise ValueError('body footprint must contain at least three XY points')
    points = tuple(tuple(float(v) for v in p) for p in points)
    if not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('body footprint must be finite')
    if not strictly_inside(0., 0., points):
        raise ValueError('body footprint must enclose base origin')
    return points


def strictly_inside(x, y, points):
    """Retain points on the boundary; do not expand the mask by any margin."""
    inside = False
    for i, (ax, ay) in enumerate(points):
        bx, by = points[(i + 1) % len(points)]
        cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
        if abs(cross) <= 1e-12 and min(ax, bx) <= x <= max(ax, bx) and min(ay, by) <= y <= max(ay, by):
            return False
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
    return inside


def mask_body_returns(ranges, angle_min, angle_increment, range_min, range_max,
                      translation, quaternion, footprint):
    if (not all(math.isfinite(v) for v in [angle_min, angle_increment, range_min, range_max, *translation, *quaternion])
            or angle_increment <= 0. or not 0. <= range_min < range_max):
        raise ValueError('invalid scan geometry or transform')
    qx, qy, qz, qw = quaternion
    if abs(sum(v * v for v in quaternion) - 1.) > 1e-5:
        raise ValueError('invalid transform quaternion')
    tx, ty, _ = translation
    output = list(ranges)
    removed = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or not range_min <= r < range_max:
            continue
        angle = angle_min + i * angle_increment
        x, y = r * math.cos(angle), r * math.sin(angle)
        bx = tx + (1. - 2. * (qy*qy + qz*qz)) * x + 2. * (qx*qy - qz*qw) * y
        by = ty + 2. * (qx*qy + qz*qw) * x + (1. - 2. * (qx*qx + qz*qz)) * y
        if inside_body_mask(bx, by, footprint):
            output[i] = float('nan')
            removed.append(i)
    return output, removed


def inside_body_mask(x, y, points):
    """Exact Euclidean 5 mm offset, including rounded polygon corners."""
    if strictly_inside(x, y, points):
        return True
    for i, (ax, ay) in enumerate(points):
        bx, by = points[(i + 1) % len(points)]
        dx, dy = bx-ax, by-ay
        length2 = dx*dx + dy*dy
        t = max(0., min(1., ((x-ax)*dx + (y-ay)*dy)/length2)) if length2 else 0.
        if math.hypot(x-(ax+t*dx), y-(ay+t*dy)) <= BODY_MASK_MARGIN_M + 1e-12:
            return True
    return False


def navigation_scan_profile(params_file, cache_dir):
    """Derive only the field-tool costmap inputs; leave source configs intact."""
    config = yaml.safe_load(Path(params_file).read_text(encoding='utf-8'))
    footprint = None
    for name, layer in [('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')]:
        params = config[name][name]['ros__parameters']
        current = footprint_points(params['footprint'])
        if footprint is not None and current != footprint:
            raise ValueError('local/global body footprints differ')
        footprint = current
        source = params[layer]['scan']
        if source['topic'] != '/scan':
            raise ValueError('unexpected navigation scan source')
        source['topic'] = NAV_SCAN_TOPIC
        # A failed filter or missing scan must make the obstacle layer stale.
        source['expected_update_rate'] = 0.5
        # Observation-layer footprint clearing cannot remove costs contributed
        # by static/inflation layers. Apply the current body + 5 mm mask last.
        params['plugins'].append('body_clear_layer')
        params['body_clear_layer'] = {'plugin': 'raicom_nav2_workspace::BodyClearLayer'}
    payload = yaml.safe_dump(config, sort_keys=False).encode('utf-8')
    folder = Path(cache_dir)
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (hashlib.sha256(payload).hexdigest() + '.yaml')
    if not path.exists() or path.read_bytes() != payload:
        fd, temporary = tempfile.mkstemp(prefix='.nav-scan-', dir=folder)
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(payload)
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return path, footprint
