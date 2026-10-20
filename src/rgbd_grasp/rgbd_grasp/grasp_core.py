"""Pure validation and waypoint helpers for one-shot RGB-D grasping."""
import math
import numpy as np


class Rejected(RuntimeError):
    """A fail-closed grasp rejection."""


def select_candidate_uvz(values, selection='nearest'):
    """Choose a nearby block; choose the upper face within the nearest depth band."""
    flat = np.asarray(values, dtype=float)
    if (flat.ndim != 1 or not len(flat) or len(flat) % 3
            or not np.all(np.isfinite(flat))):
        raise Rejected('invalid blue cube candidates')
    blocks = flat.reshape(-1, 3)
    if np.any(blocks[:, 2] <= 0.0):
        raise Rejected('invalid blue cube depth')
    if selection == 'nearest':
        nearby = blocks[blocks[:, 2] <= np.min(blocks[:, 2]) + 0.03]
        selected = nearby[np.argmin(nearby[:, 1])]
    elif selection == 'min_v':
        selected = blocks[np.argmin(blocks[:, 1])]
    elif selection == 'max_v':
        selected = blocks[np.argmax(blocks[:, 1])]
    else:
        raise Rejected('invalid target selection')
    return selected.copy(), len(blocks)

def desired_sag_compensation(base_sag_m, radius_m):
    """Increase requested Z allowance with target reach, up to 40 mm."""
    if (not math.isfinite(base_sag_m) or not 0 <= base_sag_m <= 0.04
            or not math.isfinite(radius_m) or radius_m < 0):
        raise Rejected('invalid radial sag inputs')
    return min(0.04, base_sag_m + 0.20 * max(0.0, radius_m - 0.20))


def finite_vector(value, length, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise Rejected(f'invalid {name}')
    return vector


def compensate_grasp_id4(pulses, offset, limits):
    """Apply an empirical command offset once, without changing the IK model."""
    values = finite_vector(pulses, 5, 'grasp pulses').copy()
    lower, upper = finite_vector(limits, 2, 'ID4 limits')
    offset = float(offset)
    if (not math.isfinite(offset) or offset != math.floor(offset)
            or not 0 <= offset <= 60):
        raise Rejected('grasp_id4_offset_pulse must be an integer from 0 to 60')
    if not 0 <= lower < upper <= 1000:
        raise Rejected('invalid ID4 limits')
    if np.any(values != np.floor(values)) or not lower <= values[3] <= upper:
        raise Rejected('invalid nominal grasp pulses')
    values[3] += offset
    if not lower <= values[3] <= upper:
        raise Rejected('compensated ID4 pulse outside servo limits')
    return values.astype(int)


def validate_commanded_fk(position, workspace):
    """Bound the rigid-model position after an intentional joint correction."""
    xyz = finite_vector(position, 3, 'commanded FK position')
    limits = finite_vector(workspace, 6, 'workspace')
    if (np.any(limits[::2] >= limits[1::2])
            or np.any(xyz < limits[::2]) or np.any(xyz > limits[1::2])):
        raise Rejected('compensated FK position outside configured workspace')
    return xyz


def smooth_joint_waypoints(pulses, reference_xyz, fk, workspace):
    """Damp middle-joint IK jitter while preserving path ends and Cartesian bounds."""
    raw = np.asarray(pulses, dtype=int)
    reference = np.asarray(reference_xyz, dtype=float)
    if (raw.ndim != 2 or raw.shape[1] != 5
            or reference.shape != (len(raw), 3)
            or not np.all(np.isfinite(reference))):
        raise Rejected('invalid planned joint path')
    result = [row.copy() for row in raw]
    accepted = 0
    for index in range(1, len(raw) - 1):
        median = np.median(raw[index - 1:index + 2, 1:4], axis=0)
        change = np.clip(0.5 * (median - raw[index, 1:4]), -15, 15)
        candidate = raw[index].copy()
        candidate[1:4] = np.rint(raw[index, 1:4] + change).astype(int)
        if np.array_equal(candidate, raw[index]):
            continue
        try:
            candidate_xyz = validate_commanded_fk(fk(candidate), workspace)
        except Rejected:
            continue
        if (np.linalg.norm(candidate_xyz - reference[index]) > 0.005
                or candidate_xyz[2] < reference[index, 2] - 0.002):
            continue
        result[index] = candidate
        accepted += 1
    return result, accepted


def stable_target(samples, now_s, frame, count, max_age_s, max_spread_m,
                  workspace):
    """Return stable target; reject stale, moving, or out-of-workspace
    data."""
    if count < 3 or len(samples) < count:
        raise Rejected('not enough target samples')
    selected = list(samples)[-count:]
    if any(item['frame'] != frame for item in selected):
        raise Rejected('target frame mismatch')
    stamps = np.asarray([item['stamp_s'] for item in selected], dtype=float)
    if np.any(~np.isfinite(stamps)) or np.any(stamps > now_s + 0.05):
        raise Rejected('invalid target timestamp')
    if np.any(np.diff(stamps) <= 0.0):
        raise Rejected('target timestamps must be distinct and increasing')
    if np.any(now_s - stamps > max_age_s):
        raise Rejected('target is stale')
    points = np.asarray([item['point'] for item in selected], dtype=float)
    if points.shape != (count, 3) or not np.all(np.isfinite(points)):
        raise Rejected('invalid target points')
    center = np.median(points, axis=0)
    spread = np.max(np.linalg.norm(points - center, axis=1))
    if spread > max_spread_m:
        raise Rejected('target is not stable')
    limits = finite_vector(workspace, 6, 'workspace')
    if (limits[0] >= limits[1] or limits[2] >= limits[3]
            or limits[4] >= limits[5]):
        raise Rejected('invalid workspace')
    if not (limits[0] <= center[0] <= limits[1]
            and limits[2] <= center[1] <= limits[3]
            and limits[4] <= center[2] <= limits[5]):
        raise Rejected(f'target outside configured workspace: XYZ={center.tolist()}, bounds={limits.tolist()}')
    return center


def interpolate(start, end, step_m):
    """Return Cartesian samples after start, including end."""
    start = finite_vector(start, 3, 'path start')
    end = finite_vector(end, 3, 'path end')
    distance = float(np.linalg.norm(end - start))
    if not math.isfinite(step_m) or not 0 < step_m <= 0.005:
        raise Rejected('Cartesian step must be in (0, 5mm]')
    count = max(1, int(math.ceil(distance / step_m)))
    return [start + (end - start) * (index / count)
            for index in range(1, count + 1)]


def side_grasp_waypoints(target, offset, approach_m, lift_m, step_m):
    """Build a radial side approach and vertical lift for a stationary base."""
    grasp = finite_vector(target, 3, 'target') + finite_vector(
        offset, 3, 'grasp offset')
    radial = grasp[:2]
    norm = float(np.linalg.norm(radial))
    if norm < 0.05:
        raise Rejected('target radial direction is undefined')
    if not 0.02 <= approach_m <= 0.10:
        raise Rejected('unsafe approach distance')
    if not 0.03 <= lift_m <= 0.10:
        raise Rejected('unsafe lift distance')
    direction = radial / norm
    pregrasp = grasp.copy()
    pregrasp[:2] -= direction * approach_m
    lifted = grasp.copy()
    lifted[2] += lift_m
    return (pregrasp, interpolate(pregrasp, grasp, step_m),
            interpolate(grasp, lifted, step_m))


def bounded_search_positions(center, bounds, step, sweeps):
    """Return an alternating, non-repeating ID1 search around the start."""
    center = int(center)
    lower, upper = map(int, bounds)
    step = int(step)
    sweeps = int(sweeps)
    if not 350 <= lower < upper <= 550 or not lower <= center <= upper:
        raise Rejected('ID1 search start is outside bounded range')
    if not 1 <= step <= 50:
        raise Rejected('invalid ID1 search step')
    if not 1 <= sweeps <= 5:
        raise Rejected('invalid ID1 search sweep count')
    positions = []
    seen = {center}
    for sweep in range(1, sweeps + 1):
        for sign in (-1, 1):
            candidate = max(lower, min(upper, center + sign * step * sweep))
            if candidate not in seen:
                positions.append(candidate)
                seen.add(candidate)
    return positions


def validate_arm_pulses(pulses, limits, id1_range, fixed_id5):
    """Validate an IK result and enforce bounded pan plus a fixed ID5."""
    values = finite_vector(pulses, 5, 'IK pulses')
    ranges = np.asarray(limits, dtype=float)
    if ranges.shape != (5, 2) or np.any(~np.isfinite(ranges)):
        raise Rejected('invalid servo limits')
    if np.any(ranges[:, 0] >= ranges[:, 1]):
        raise Rejected('invalid servo limit order')
    if np.any(values != np.floor(values)):
        raise Rejected('IK returned fractional pulses')
    values = values.astype(int)
    lower, upper = map(int, id1_range)
    if not 350 <= lower < upper <= 550 or not lower <= values[0] <= upper:
        raise Rejected('ID1 outside bounded range')
    values[4] = int(fixed_id5)
    if np.any(values < ranges[:, 0]) or np.any(values > ranges[:, 1]):
        raise Rejected('IK pulse outside servo limits')
    return values


def validate_joint_step(previous, current, maximum):
    previous = finite_vector(previous, 5, 'previous joints')
    current = finite_vector(current, 5, 'current joints')
    if not math.isfinite(maximum) or maximum <= 0:
        raise Rejected('invalid joint jump limit')
    if np.max(np.abs(current - previous)) > maximum:
        raise Rejected('IK joint jump exceeds limit')
