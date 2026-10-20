"""Offline data-model tests for the field mapping and navigation tool."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from raicom_control.field_tool_model import (
    atomic_write_route,
    delete_point,
    discover_maps,
    migrate_legacy_route,
    navigation_pose,
    new_route,
    next_map_name,
    read_route_file,
    record_point,
    route_file_for_map,
    set_approach_offset,
    set_order,
    set_point_tolerances,
)


class FieldToolModelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.maps = self.root / 'maps'
        self.routes = self.root / 'routes'
        self.maps.mkdir()
        self.routes.mkdir()
        self.map_file = self.maps / '20260912_020000_field.yaml'
        self.image = self.maps / '20260912_020000_field.pgm'
        self.image.write_bytes(b'P5\n# generated\n2 2\n255\n\x00\xfe\xcd\xfe')
        self.map_file.write_text(
            'image: 20260912_020000_field.pgm\n'
            'mode: trinary\n'
            'resolution: 0.02\n'
            'origin: [0.0, 0.0, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n',
            encoding='utf-8',
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_point_lifecycle_and_approach(self):
        route = new_route(self.map_file)
        record_point(route, 'start', '启动区', 0.1, 0.2, math.radians(90))
        record_point(route, 'pickup', '夹取区', 1.0, 0.0, 0.0)
        self.assertEqual(route['order'], ['start', 'pickup'])
        set_approach_offset(route, 'start', 0.2, -0.1)
        start = route['points']['start']
        self.assertAlmostEqual(start['approach']['x'], 0.2)
        self.assertAlmostEqual(start['approach']['y'], 0.4)
        self.assertAlmostEqual(start['approach']['yaw_rad'], math.radians(90))
        set_point_tolerances(route, 'pickup', 0.08, 6.0)
        self.assertEqual(route['points']['pickup']['position_tolerance_m'], 0.08)
        self.assertEqual(
            navigation_pose(route['points']['pickup']),
            route['points']['pickup']['approach'])

    def test_reference_correction_moves_approach_and_preserves_history(self):
        route = new_route(self.map_file)
        record_point(route, 'start', '启动区', 0.0, 0.0, 0.0)
        set_approach_offset(route, 'start', 0.1, 0.0)
        old_approach = deepcopy(route['points']['start']['approach'])
        record_point(route, 'start', '启动区', 1.0, 0.0, math.radians(90))
        point = route['points']['start']
        self.assertAlmostEqual(point['approach']['x'], 1.0)
        self.assertAlmostEqual(point['approach']['y'], 0.1)
        self.assertAlmostEqual(point['approach']['yaw_rad'], math.radians(90))
        self.assertNotEqual(point['approach'], old_approach)
        self.assertTrue(any(item['kind'] == 'approach' for item in route['history']))

    def test_route_file_is_bound_to_exact_map(self):
        route = new_route(self.map_file)
        record_point(route, 'start', '启动区', 0.0, 0.0, 0.0)
        route_path = route_file_for_map(self.map_file, self.routes)
        atomic_write_route(route_path, route)
        loaded = read_route_file(route_path, self.map_file)
        self.assertEqual(loaded['points']['start']['label'], '启动区')
        self.image.write_bytes(b'P5\n# changed\n2 2\n255\n\xff\x01\x02\x03')
        with self.assertRaisesRegex(ValueError, 'different saved map'):
            read_route_file(route_path, self.map_file)

    def test_order_delete_and_atomic_failure(self):
        route = new_route(self.map_file)
        record_point(route, 'a', 'A', 0, 0, 0)
        record_point(route, 'b', 'B', 1, 0, 0)
        set_order(route, ['b', 'a'])
        self.assertEqual(route['order'], ['b', 'a'])
        delete_point(route, 'a')
        self.assertEqual(route['order'], ['b'])
        route_path = route_file_for_map(self.map_file, self.routes)
        atomic_write_route(route_path, route)
        before = route_path.read_text(encoding='utf-8')
        with self.assertRaises(ValueError):
            atomic_write_route(route_path, {'schema': 'broken', 'nan': float('nan')})
        self.assertEqual(route_path.read_text(encoding='utf-8'), before)
        self.assertFalse(list(self.routes.glob('*.tmp')))

    def test_discover_and_unique_name(self):
        self.assertEqual(discover_maps(self.maps), [self.map_file.resolve()])
        first = next_map_name(self.maps, 'Field One')
        (self.maps / f'{first}.yaml').write_text('x', encoding='utf-8')
        second = next_map_name(self.maps, 'Field One')
        self.assertNotEqual(first, second)

    def test_reload_preserves_offset_and_allows_reference_correction(self):
        route = new_route(self.map_file)
        record_point(route, 'start', 'Start', 0, 0, 0)
        set_approach_offset(route, 'start', .2, .1)
        path = route_file_for_map(self.map_file, self.routes)
        atomic_write_route(path, route)
        loaded = read_route_file(path, self.map_file)
        self.assertEqual(loaded['points']['start']['approach']['offset_from_reference_m'], [.2, .1])
        record_point(loaded, 'start', 'Start', 1, 2, math.pi / 2)
        pose = loaded['points']['start']['approach']
        self.assertAlmostEqual(pose['x'], .9)
        self.assertAlmostEqual(pose['y'], 2.2)

    def test_resurvey_approach_updates_relative_offset(self):
        route = new_route(self.map_file)
        record_point(route, 'start', 'Start', 1, 2, math.pi / 2)
        record_point(route, 'start', 'Start', .9, 2.2, math.pi / 2, kind='approach')
        record_point(route, 'start', 'Start', 2, 3, 0)
        pose = route['points']['start']['approach']
        self.assertAlmostEqual(pose['x'], 2.2)
        self.assertAlmostEqual(pose['y'], 3.1)

    def test_invalid_edits_leave_point_and_history_unchanged(self):
        route = new_route(self.map_file)
        record_point(route, 'a', 'A', 0, 0, 0)
        before = deepcopy(route)
        for edit in (lambda: set_point_tolerances(route, 'a', 1, 5),
                     lambda: record_point(route, 'a', 'A', 2, 0, 0, kind='approach')):
            with self.assertRaises(ValueError):
                edit()
            self.assertEqual(route, before)

    def test_same_filename_different_map_cannot_overwrite_route(self):
        route = new_route(self.map_file)
        path = route_file_for_map(self.map_file, self.routes)
        atomic_write_route(path, route)
        before = path.read_bytes()
        self.image.write_bytes(b'P5\n2 2\n255\n\xff\xff\x00\x00')
        other = new_route(self.map_file)
        self.assertNotEqual(path, route_file_for_map(self.map_file, self.routes))
        with self.assertRaisesRegex(ValueError, 'different saved map'):
            atomic_write_route(path, other)
        self.assertEqual(path.read_bytes(), before)

    def test_legacy_route_can_be_imported_without_overwriting_it(self):
        config = self.root / 'config'
        config.mkdir()
        new = new_route(self.map_file)
        legacy = {
            'schema': 'raicom_manual_route/v1',
            'frame': 'map',
            'map': new['map'],
            'points': {
                'start': {
                    'label': '启动区',
                    'x': 0.0,
                    'y': 0.0,
                    'yaw_rad': 0.0,
                    'yaw_deg': 0.0,
                    'recorded_at_utc': '2026-01-01T00:00:00+00:00',
                }
            },
            'history': [],
        }
        legacy_path = config / 'route_20260912_020000_field.json'
        legacy_path.write_text(
            json.dumps(legacy, ensure_ascii=False), encoding='utf-8')
        raw = legacy_path.read_bytes()
        approach = {
            'schema': 'raicom_navigation_approach/v2',
            'route_sha256': hashlib.sha256(raw).hexdigest(),
            'map_yaml_sha256': new['map']['saved']['yaml_sha256'],
            'points': {
                'start': {
                    'offset_from_work_pose_m': [0.1, 0.0],
                }
            },
        }
        approach_path = (
            config / 'navigation_approach_20260912_020000_field.json')
        approach_path.write_text(
            json.dumps(approach, ensure_ascii=False), encoding='utf-8')
        before = legacy_path.read_bytes()
        migrated = migrate_legacy_route(self.map_file, [config])
        self.assertIsNotNone(migrated)
        self.assertAlmostEqual(
            migrated['points']['start']['approach']['x'], 0.1)
        self.assertEqual(legacy_path.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
