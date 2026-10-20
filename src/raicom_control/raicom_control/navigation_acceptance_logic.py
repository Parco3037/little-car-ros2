"""Pure fail-closed logic for one explicitly authorized navigation leg."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import yaml

from .route_points_logic import read_route, saved_map_binding


APPROACH_SCHEMA = 'raicom_navigation_approach/v2'
MAX_APPROACH_OFFSET_M = 0.20
MAX_ROTATION_SWEEP_STEP_RAD = math.radians(5.0)
REQUIRED_MISSION_LEGS = {
    ('start', 'recognition'),
    ('recognition', 'pickup'),
    ('pickup', 'park_1'),
    ('pickup', 'park_2'),
    ('park_1', 'pickup'),
    ('park_1', 'start'),
    ('park_2', 'start'),
}


@dataclass(frozen=True)
class LegSpec:
    source_name: str
    target_name: str
    source: dict
    target: dict
    route_sha256: str
    loaded_map_binding: dict
    work_target: dict | None = None
    approach_sha256: str | None = None
    footprint: dict | None = None
    static_checks: dict | None = None
    rotation_checks: dict | None = None
    source_reference_kind: str = 'work_pose'


def _pgm_pixels(path):
    """Read the preserved 8-bit binary PGM without image-library dependencies."""
    raw = Path(path).read_bytes()
    index = 0

    def token():
        nonlocal index
        while index < len(raw):
            if raw[index] == ord('#'):
                newline = raw.find(b'\n', index)
                if newline < 0:
                    raise ValueError('Unterminated PGM comment')
                index = newline + 1
            elif chr(raw[index]).isspace():
                index += 1
            else:
                break
        start = index
        while index < len(raw) and not chr(raw[index]).isspace() and raw[index] != ord('#'):
            index += 1
        if start == index:
            raise ValueError('Missing PGM header token')
        return raw[start:index]

    if token() != b'P5':
        raise ValueError('Only binary P5 PGM maps are supported')
    width, height, maximum = int(token()), int(token()), int(token())
    if width <= 0 or height <= 0 or not 0 < maximum < 256:
        raise ValueError('Unsupported PGM dimensions or sample depth')
    if index >= len(raw) or not chr(raw[index]).isspace():
        raise ValueError('Missing PGM raster delimiter')
    if raw[index:index + 2] == b'\r\n':
        index += 2
    else:
        index += 1
    pixels = raw[index:index + width * height]
    if len(pixels) != width * height or index + len(pixels) != len(raw):
        raise ValueError('PGM raster size mismatch')
    return width, height, maximum, pixels


def _pad_footprint(points, padding):
    """Mirror Nav2 Humble padFootprint(): add signed padding per coordinate."""
    padded = []
    for x, y in points:
        x, y = float(x), float(y)
        padded.append((x + math.copysign(padding, x) if x else x,
                       y + math.copysign(padding, y) if y else y))
    return padded


def _validated_footprint(footprint):
    if not isinstance(footprint, dict):
        raise ValueError('Navigation footprint is invalid')
    raw_points = footprint.get('points_xy_m')
    try:
        padding = float(footprint.get('padding_m', 0.0))
    except (TypeError, ValueError) as exc:
        raise ValueError('Navigation footprint padding is invalid') from exc
    if (not isinstance(raw_points, list) or len(raw_points) < 3
            or not math.isfinite(padding) or not 0.0 <= padding <= 0.10):
        raise ValueError('Navigation footprint or padding is invalid')
    points = []
    for raw_point in raw_points:
        if not isinstance(raw_point, list) or len(raw_point) != 2:
            raise ValueError('Navigation footprint point is invalid')
        try:
            point = tuple(float(value) for value in raw_point)
        except (TypeError, ValueError) as exc:
            raise ValueError('Navigation footprint point is invalid') from exc
        if not all(math.isfinite(value) for value in point):
            raise ValueError('Navigation footprint point is non-finite')
        points.append(point)
    turns = []
    for index, point in enumerate(points):
        previous = points[index - 1]
        following = points[(index + 1) % len(points)]
        turns.append(
            (point[0] - previous[0]) * (following[1] - point[1])
            - (point[1] - previous[1]) * (following[0] - point[0]))
    if not all(turn > 1e-12 for turn in turns):
        raise ValueError('Navigation footprint must be strictly convex and counter-clockwise')
    return points, padding


def _polygon_intersects_cell(polygon, center_x, center_y, half_cell):
    square = (
        (center_x - half_cell, center_y - half_cell),
        (center_x + half_cell, center_y - half_cell),
        (center_x + half_cell, center_y + half_cell),
        (center_x - half_cell, center_y + half_cell),
    )
    axes = [(1.0, 0.0), (0.0, 1.0)]
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        edge_x, edge_y = second[0] - first[0], second[1] - first[1]
        axes.append((-edge_y, edge_x))
    for axis_x, axis_y in axes:
        robot = [x * axis_x + y * axis_y for x, y in polygon]
        cell = [x * axis_x + y * axis_y for x, y in square]
        if max(robot) < min(cell) - 1e-12 or max(cell) < min(robot) - 1e-12:
            return False
    return True


def static_pose_collision_cells(map_file, footprint, pose):
    """Return blocked saved-map cells touched by a padded oriented footprint."""
    map_path = Path(map_file).resolve(strict=True)
    metadata = yaml.safe_load(map_path.read_text(encoding='utf-8'))
    if metadata.get('mode') != 'trinary' or int(metadata.get('negate', -1)) != 0:
        raise ValueError('Only non-negated trinary saved maps are supported')
    resolution = float(metadata['resolution'])
    origin_x, origin_y, origin_yaw = map(float, metadata['origin'])
    if resolution <= 0.0 or abs(origin_yaw) > 1e-12:
        raise ValueError('Saved map resolution/origin is unsupported')
    image_path = (map_path.parent / metadata['image']).resolve(strict=True)
    image_path.relative_to(map_path.parent)
    width, height, maximum, pixels = _pgm_pixels(image_path)
    points, padding = _validated_footprint(footprint)
    padded = _pad_footprint(points, padding)
    try:
        pose_x, pose_y, yaw = map(float, (pose['x'], pose['y'], pose['yaw_rad']))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Navigation pose is invalid') from exc
    if not all(math.isfinite(value) for value in (pose_x, pose_y, yaw)):
        raise ValueError('Navigation pose is non-finite')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    robot = [
        (pose_x + x * cosine - y * sine,
         pose_y + x * sine + y * cosine)
        for x, y in padded
    ]
    lower_x, upper_x = min(x for x, _ in robot), max(x for x, _ in robot)
    lower_y, upper_y = min(y for _, y in robot), max(y for _, y in robot)
    free_threshold = float(metadata['free_thresh'])
    collisions = []
    half = resolution / 2.0
    for row in range(height):
        center_y = origin_y + (height - 1 - row + 0.5) * resolution
        if center_y + half < lower_y or center_y - half > upper_y:
            continue
        for column in range(width):
            center_x = origin_x + (column + 0.5) * resolution
            if center_x + half < lower_x or center_x - half > upper_x:
                continue
            occupancy = (maximum - pixels[row * width + column]) / maximum
            # Unknown cells are deliberately blocked for acceptance goals.
            if occupancy < free_threshold:
                continue
            if _polygon_intersects_cell(robot, center_x, center_y, half):
                collisions.append({
                    'row': row,
                    'column': column,
                    'x': center_x,
                    'y': center_y,
                    'occupancy': occupancy,
                })
    return collisions


def _rotation_sweep_yaws(source_yaw, target_yaw, max_step_rad):
    try:
        source_yaw = float(source_yaw)
        target_yaw = float(target_yaw)
        max_step_rad = float(max_step_rad)
    except (TypeError, ValueError) as exc:
        raise ValueError('Terminal rotation sweep is invalid') from exc
    if (not all(math.isfinite(value) for value in (
            source_yaw, target_yaw, max_step_rad))
            or not 0.0 < max_step_rad <= MAX_ROTATION_SWEEP_STEP_RAD + 1e-12):
        raise ValueError('Terminal rotation sweep step is invalid')
    delta = wrap_angle(target_yaw - source_yaw)
    segment_count = max(1, math.ceil(abs(delta) / max_step_rad))
    return [source_yaw + delta * index / segment_count
            for index in range(segment_count + 1)]


def terminal_rotation_sweep_check(
        map_file, footprint, target_pose, source_yaw, max_step_rad):
    """Check every sampled orientation used by a shortest terminal rotation."""
    target_yaw = float(target_pose['yaw_rad'])
    samples = _rotation_sweep_yaws(source_yaw, target_yaw, max_step_rad)
    blocked_samples = []
    unique_cells = set()
    intersections = 0
    for yaw in samples:
        collisions = static_pose_collision_cells(
            map_file, footprint, dict(target_pose, yaw_rad=yaw))
        if not collisions:
            continue
        intersections += len(collisions)
        unique_cells.update((row['row'], row['column']) for row in collisions)
        blocked_samples.append({
            'yaw_rad': yaw,
            'blocked_cell_intersections': len(collisions),
        })
    return {
        'source_yaw_rad': float(source_yaw),
        'target_yaw_rad': target_yaw,
        'max_step_rad': float(max_step_rad),
        'sample_count': len(samples),
        'blocked_sample_count': len(blocked_samples),
        'blocked_cell_intersections': intersections,
        'unique_blocked_cells': len(unique_cells),
        'first_blocked_sample': blocked_samples[0] if blocked_samples else None,
    }


def _load_approach_spec(approach_file, route_sha256, map_file, route_points):
    path = Path(approach_file).resolve(strict=True)
    raw = path.read_bytes()
    document = json.loads(raw)
    if document.get('schema') != APPROACH_SCHEMA:
        raise ValueError('Navigation approach schema is invalid')
    if document.get('route_sha256') != route_sha256:
        raise ValueError('Navigation approach is not bound to the selected route')
    map_digest = hashlib.sha256(Path(map_file).resolve(strict=True).read_bytes()).hexdigest()
    if document.get('map_yaml_sha256') != map_digest:
        raise ValueError('Navigation approach is not bound to the selected map YAML')
    points = document.get('points')
    if not isinstance(points, dict) or set(points) != set(route_points):
        raise ValueError('Navigation approach station set differs from the route')
    footprint = document.get('footprint')
    _validated_footprint(footprint)
    checks = {}
    for name, pose in points.items():
        if pose.get('work_pose') != name:
            raise ValueError(f'Navigation approach work-pose binding is invalid: {name}')
        offset = pose.get('offset_from_work_pose_m')
        work = route_points[name]
        try:
            values = tuple(map(float, (
                offset[0], offset[1], pose['x'], pose['y'], pose['yaw_rad'])))
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise ValueError(f'Navigation approach offset is invalid: {name}') from exc
        offset_x, offset_y, pose_x, pose_y, pose_yaw = values
        if (not isinstance(offset, list) or len(offset) != 2
                or not all(math.isfinite(value) for value in values)
                or math.hypot(offset_x, offset_y) > MAX_APPROACH_OFFSET_M
                or abs(float(work['x']) + offset_x - pose_x) > 1e-9
                or abs(float(work['y']) + offset_y - pose_y) > 1e-9
                or abs(wrap_angle(float(work['yaw_rad']) - pose_yaw)) > 1e-9):
            raise ValueError(f'Navigation approach offset is invalid: {name}')
        collisions = static_pose_collision_cells(map_file, footprint, pose)
        checks[name] = {'blocked_cell_intersections': len(collisions)}
        if collisions:
            raise ValueError(
                f'Navigation approach {name} intersects {len(collisions)} blocked map cells')
    try:
        sweep_step = float(document['terminal_rotation_sweep_step_rad'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Terminal rotation sweep step is missing or invalid') from exc
    # Validate the step even before a map cell is inspected.
    _rotation_sweep_yaws(0.0, 0.0, sweep_step)
    raw_legs = document.get('mission_legs')
    if not isinstance(raw_legs, list):
        raise ValueError('Navigation mission legs are missing or invalid')
    mission_legs = []
    for row in raw_legs:
        if (not isinstance(row, dict)
                or set(row) != {'source', 'target', 'source_reference'}
                or not isinstance(row['source'], str)
                or not isinstance(row['target'], str)
                or row['source_reference'] not in {'work_pose', 'approach_pose'}):
            raise ValueError('Navigation mission leg is invalid')
        leg = (row['source'], row['target'])
        if (leg[0] not in points or leg[1] not in points or leg[0] == leg[1]
                or leg in mission_legs):
            raise ValueError(f'Navigation mission leg is invalid: {leg!r}')
        mission_legs.append(leg)
    if set(mission_legs) != REQUIRED_MISSION_LEGS:
        raise ValueError('Navigation mission leg set is incomplete or unexpected')
    if next(row for row in raw_legs
            if (row['source'], row['target']) == ('start', 'recognition'))[
                'source_reference'] != 'work_pose':
        raise ValueError('Initial start leg must use the recorded work pose as its source')
    rotation_checks = {}
    for source_name, target_name in mission_legs:
        key = f'{source_name}->{target_name}'
        check = terminal_rotation_sweep_check(
            map_file, footprint, points[target_name],
            points[source_name]['yaw_rad'], sweep_step)
        rotation_checks[key] = check
        if check['blocked_sample_count']:
            raise ValueError(
                f'Navigation terminal rotation {key} intersects blocked map cells: '
                f"{check['blocked_sample_count']} sampled orientations, "
                f"{check['unique_blocked_cells']} unique cells")
    return document, hashlib.sha256(raw).hexdigest(), checks, rotation_checks


def load_leg_spec(route_file, map_file, source_name, target_name, approach_file=None):
    """Load a named leg only when the journal is bound to the saved map."""
    route_path = Path(route_file).resolve(strict=True)
    raw = route_path.read_bytes()
    document = json.loads(raw)
    binding = document.get('map') if isinstance(document, dict) else None
    document = read_route(raw.decode('utf-8'), binding)
    actual_saved = saved_map_binding(map_file)
    if document['map'].get('saved') != actual_saved:
        raise ValueError('Route saved-map hashes or metadata do not match the selected map')
    if source_name == target_name:
        raise ValueError('Source and target station must differ')
    for name in (source_name, target_name):
        if name not in document['points']:
            raise ValueError(f'Route station is not recorded: {name}')
    source, target = document['points'][source_name], document['points'][target_name]
    work_target = None
    approach_sha256 = None
    footprint = None
    static_checks = None
    rotation_checks = None
    source_reference_kind = 'work_pose'
    if approach_file is not None:
        approach, approach_sha256, static_checks, rotation_checks = _load_approach_spec(
            approach_file, hashlib.sha256(raw).hexdigest(), map_file, document['points'])
        leg_contract = next(
            row for row in approach['mission_legs']
            if row['source'] == source_name and row['target'] == target_name)
        source_reference_kind = leg_contract['source_reference']
        source = (document['points'][source_name]
                  if source_reference_kind == 'work_pose'
                  else approach['points'][source_name])
        target = approach['points'][target_name]
        work_target = document['points'][target_name]
        footprint = approach['footprint']
    return LegSpec(
        source_name=source_name,
        target_name=target_name,
        source=source,
        target=target,
        route_sha256=hashlib.sha256(raw).hexdigest(),
        loaded_map_binding=document['map']['loaded'],
        work_target=work_target,
        approach_sha256=approach_sha256,
        footprint=footprint,
        static_checks=static_checks,
        rotation_checks=rotation_checks,
        source_reference_kind=source_reference_kind,
    )


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def source_pose_errors(current_x, current_y, current_yaw, source):
    position = math.hypot(float(current_x) - float(source['x']),
                          float(current_y) - float(source['y']))
    heading = abs(wrap_angle(float(current_yaw) - float(source['yaw_rad'])))
    return position, heading


def load_resume_reference(evidence_file, spec):
    """Bind an interrupted continuation to the last pose of one failed leg."""
    path = Path(evidence_file).resolve(strict=True)
    raw = path.read_bytes()
    document = json.loads(raw)
    if document.get('schema') != 'raicom_navigation_acceptance/v1':
        raise ValueError('Resume evidence schema is invalid')
    if document.get('phase') != 'terminal' or document.get('outcome') == 'SUCCEEDED':
        raise ValueError('Resume evidence is not a terminal failed leg')
    if document.get('stop_latch_published') is not True:
        raise ValueError('Resume evidence does not prove a stop latch')
    leg = document.get('leg', {})
    expected = (spec.source_name, spec.target_name, spec.route_sha256)
    actual = (leg.get('source'), leg.get('target'), leg.get('route_sha256'))
    if actual != expected:
        raise ValueError('Resume evidence belongs to a different navigation leg')
    if leg.get('approach_sha256') != spec.approach_sha256:
        raise ValueError('Resume evidence belongs to a different navigation approach')
    feedback = document.get('feedback')
    if not isinstance(feedback, list) or not feedback:
        raise ValueError('Resume evidence has no terminal feedback pose')
    pose = feedback[-1].get('pose', {})
    values = [float(pose.get(key, math.nan)) for key in ('x', 'y', 'z', 'w')]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Resume evidence terminal pose is invalid')
    x, y, z, w = values
    return {
        'kind': 'failed_leg_terminal_feedback',
        'x': x,
        'y': y,
        'yaw_rad': wrap_angle(2.0 * math.atan2(z, w)),
        'evidence_sha256': hashlib.sha256(raw).hexdigest(),
        'evidence_finished_at_utc': document.get('finished_at_utc'),
        'prior_outcome': document.get('outcome'),
        'prior_abort_reason': document.get('abort_reason'),
    }


def graph_preflight_errors(graph):
    """Return deterministic ROS graph ownership errors."""
    expected = {
        'controller_publishers': ['cmd_vel_guard'],
        'staging_publishers': ['velocity_smoother'],
        'staging_subscribers': ['nav_cmd_bridge'],
    }
    errors = []
    for field, owners in expected.items():
        actual = sorted(graph.get(field, []))
        if actual != owners:
            errors.append(f'{field}={actual!r}, expected={owners!r}')
    if len(graph.get('localization_publishers', [])) != 1:
        errors.append(
            'localization_publishers must contain exactly one live owner: '
            f"{sorted(graph.get('localization_publishers', []))!r}"
        )
    if graph.get('map_publishers') != ['map_server']:
        errors.append(
            f"map_publishers={sorted(graph.get('map_publishers', []))!r}, "
            "expected=['map_server']"
        )
    return errors


@dataclass
class LegProgressMonitor:
    started_at: float
    max_recoveries: int = 2
    feedback_timeout_sec: float = 2.0
    progress_timeout_sec: float = 8.0
    leg_timeout_sec: float = 60.0
    min_progress_m: float = 0.02
    progress_exempt_distance_m: float = 0.08

    def __post_init__(self):
        self.last_feedback_at = self.started_at
        self.last_progress_at = self.started_at
        self.best_distance = math.inf
        self.last_recoveries = 0

    def observe(self, now, distance_remaining, recoveries):
        distance = float(distance_remaining)
        recovery_count = int(recoveries)
        if not math.isfinite(distance) or distance < 0.0 or recovery_count < 0:
            return 'invalid_feedback'
        self.last_feedback_at = float(now)
        self.last_recoveries = recovery_count
        if recovery_count > self.max_recoveries:
            return f'too_many_recoveries:{recovery_count}>{self.max_recoveries}'
        if not math.isfinite(self.best_distance) or distance <= self.best_distance - self.min_progress_m:
            self.best_distance = distance
            self.last_progress_at = float(now)
        elif distance < self.best_distance:
            self.best_distance = distance
        return self.check(now)

    def check(self, now):
        now = float(now)
        if now - self.started_at > self.leg_timeout_sec:
            return f'leg_timeout:{now - self.started_at:.3f}s'
        if now - self.last_feedback_at > self.feedback_timeout_sec:
            return f'feedback_stale:{now - self.last_feedback_at:.3f}s'
        if (self.best_distance > self.progress_exempt_distance_m and
                now - self.last_progress_at > self.progress_timeout_sec):
            return f'no_progress:{now - self.last_progress_at:.3f}s'
        return None


@dataclass(frozen=True)
class CommandResponseConfig:
    """Fail-closed thresholds for commanded motion versus map-localized pose."""

    linear_threshold_mps: float = 0.03
    angular_threshold_radps: float = 0.08
    equivalent_turn_radius_m: float = 0.15
    observation_window_sec: float = 3.0
    command_gap_sec: float = 0.35
    min_active_samples: int = 20
    min_command_effort_m: float = 0.08
    min_pose_response_m: float = 0.005
    min_response_ratio: float = 0.10
    max_sample_dt_sec: float = 0.25

    def __post_init__(self):
        positive = (
            self.linear_threshold_mps,
            self.angular_threshold_radps,
            self.equivalent_turn_radius_m,
            self.observation_window_sec,
            self.command_gap_sec,
            self.min_command_effort_m,
            self.min_pose_response_m,
            self.min_response_ratio,
            self.max_sample_dt_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError('Command-response thresholds must be finite and positive')
        if self.min_active_samples <= 0:
            raise ValueError('min_active_samples must be positive')


class CommandPoseResponseMonitor:
    """Detect sustained commands with inadequate map-localized pose response.

    Vendor ``/odom_raw`` is open-loop command integration and cannot prove
    physical motion.  This monitor instead consumes successive, independently
    timestamped ``map -> base_footprint`` poses.  Linear distance and yaw travel
    are combined with a conservative 0.15 m equivalent turn radius.
    """

    def __init__(self, config=None):
        self.config = config or CommandResponseConfig()
        self.armed_at = None
        self.last_pose_at = None
        self.last_pose = None
        self.max_pose_equivalent_step_m = 0.0
        self.last_reason = None
        self._reset_segment()

    def _reset_segment(self):
        self.active_started_at = None
        self.last_command_at = None
        self.last_command_equivalent_mps = 0.0
        self.active_samples = 0
        self.command_effort_m = 0.0
        self.pose_response_m = 0.0
        self.max_command_equivalent_mps = 0.0

    def arm(self, now, map_pose=None):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('arm time must be finite')
        self.armed_at = now
        self.last_reason = None
        self._reset_segment()
        self.last_pose_at = now if map_pose is not None else None
        self.last_pose = None
        if map_pose is not None:
            self.last_pose = self._finite_values(map_pose, 'map pose')
        self.max_pose_equivalent_step_m = 0.0

    @staticmethod
    def _finite_values(values, label):
        result = tuple(float(value) for value in values)
        if any(not math.isfinite(value) for value in result):
            raise ValueError(f'{label} must contain finite values')
        return result

    @staticmethod
    def _yaw_delta(current, previous):
        return abs(math.atan2(math.sin(current - previous), math.cos(current - previous)))

    def _command_values(self, linear_x, linear_y, angular_z):
        linear_x, linear_y, angular_z = self._finite_values(
            (linear_x, linear_y, angular_z), 'command')
        linear = math.hypot(linear_x, linear_y)
        equivalent = linear + self.config.equivalent_turn_radius_m * abs(angular_z)
        active = (linear >= self.config.linear_threshold_mps or
                  abs(angular_z) >= self.config.angular_threshold_radps)
        return equivalent, active

    def observe_command(self, now, linear_x, linear_y, angular_z):
        if self.armed_at is None:
            return None
        now = float(now)
        if not math.isfinite(now):
            return 'invalid_command_observation'
        try:
            equivalent, active = self._command_values(linear_x, linear_y, angular_z)
        except ValueError:
            return 'invalid_command_observation'

        if not active:
            reason = self.check(now)
            if reason is None:
                self._reset_segment()
            return reason

        if self.active_started_at is None:
            self._reset_segment()
            self.active_started_at = now
        elif (self.last_command_at is not None and
              now - self.last_command_at > self.config.command_gap_sec):
            reason = self.check(now)
            if reason is not None:
                return reason
            self._reset_segment()
            self.active_started_at = now
        elif self.last_command_at is not None:
            dt = min(max(0.0, now - self.last_command_at),
                     self.config.max_sample_dt_sec)
            self.command_effort_m += self.last_command_equivalent_mps * dt

        self.last_command_at = now
        self.last_command_equivalent_mps = equivalent
        self.active_samples += 1
        self.max_command_equivalent_mps = max(
            self.max_command_equivalent_mps, equivalent)
        return self.check(now)

    def observe_pose(self, now, x, y, yaw):
        if self.armed_at is None:
            return None
        now = float(now)
        if not math.isfinite(now):
            return 'invalid_map_pose_observation'
        try:
            pose = self._finite_values((x, y, yaw), 'map pose')
        except ValueError:
            return 'invalid_map_pose_observation'

        if (self.last_pose is not None and self.active_started_at is not None and
                self.last_command_at is not None and
                0.0 <= now - self.last_command_at <= self.config.command_gap_sec):
            dx = pose[0] - self.last_pose[0]
            dy = pose[1] - self.last_pose[1]
            dyaw = self._yaw_delta(pose[2], self.last_pose[2])
            equivalent_step = (
                math.hypot(dx, dy) + self.config.equivalent_turn_radius_m * dyaw)
            self.pose_response_m += equivalent_step
            self.max_pose_equivalent_step_m = max(
                self.max_pose_equivalent_step_m, equivalent_step)
        self.last_pose_at = now
        self.last_pose = pose
        return self.check(now)

    def check(self, now):
        if self.armed_at is None or self.active_started_at is None:
            return None
        now = float(now)
        if not math.isfinite(now):
            return 'invalid_motion_response_time'
        if (self.last_command_at is not None and
                now - self.last_command_at > self.config.command_gap_sec):
            reason = self._evaluate(self.last_command_at)
            if reason is None:
                self._reset_segment()
            return reason
        return self._evaluate(now)

    def _evaluate(self, now):
        active_duration = max(0.0, float(now) - self.active_started_at)
        if (active_duration < self.config.observation_window_sec or
                self.active_samples < self.config.min_active_samples or
                self.command_effort_m < self.config.min_command_effort_m):
            return None
        required = max(
            self.config.min_pose_response_m,
            self.config.min_response_ratio * self.command_effort_m,
        )
        if self.pose_response_m >= required:
            return None
        ratio = self.pose_response_m / self.command_effort_m
        self.last_reason = (
            'commanded_motion_without_map_pose_response:'
            f'duration={active_duration:.3f}s,'
            f'command={self.command_effort_m:.4f}m,'
            f'map_pose={self.pose_response_m:.4f}m,'
            f'ratio={ratio:.3f}<{self.config.min_response_ratio:.3f}'
        )
        return self.last_reason

    def snapshot(self, now=None):
        if now is None:
            now = self.last_command_at if self.last_command_at is not None else self.armed_at
        now = None if now is None else float(now)
        elapsed = None
        if now is not None and self.active_started_at is not None:
            elapsed = max(0.0, now - self.active_started_at)
        ratio = None
        if self.command_effort_m > 0.0:
            ratio = self.pose_response_m / self.command_effort_m
        return {
            'armed': self.armed_at is not None,
            'active': self.active_started_at is not None,
            'active_duration_sec': elapsed,
            'active_samples': self.active_samples,
            'command_effort_equivalent_m': self.command_effort_m,
            'map_pose_response_equivalent_m': self.pose_response_m,
            'response_ratio': ratio,
            'max_command_equivalent_mps': self.max_command_equivalent_mps,
            'max_map_pose_equivalent_step_m': self.max_pose_equivalent_step_m,
            'last_command_age_sec': (
                None if now is None or self.last_command_at is None
                else max(0.0, now - self.last_command_at)),
            'last_map_pose_age_sec': (
                None if now is None or self.last_pose_at is None
                else max(0.0, now - self.last_pose_at)),
            'last_reason': self.last_reason,
            'config': {
                field: getattr(self.config, field)
                for field in self.config.__dataclass_fields__
            },
        }
