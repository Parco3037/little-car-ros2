"""Offline tests: launch actions are captured, never executed and never publish."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
import yaml

from raicom_control.map_preflight import validate_saved_map


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'localization_entry', ROOT / 'launch' / 'home_localization.launch.py')
ENTRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTRY)


class LocalizationEntryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='raicom_localization_test_')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.map_file = self.root / 'map_01.yaml'
        self.image = self.root / 'map_01.pgm'
        self.metadata = {
            'image': self.image.name, 'mode': 'trinary', 'resolution': 0.05,
            'origin': [-1.16, -1.91, 0.0], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.25,
        }
        self.image.write_bytes(b'P5\n# map_saver output\n2 2\n255\n\x00\xfe\xcd\xfe')
        self.write_metadata()

    def write_metadata(self):
        self.map_file.write_text(yaml.safe_dump(self.metadata), encoding='utf-8')

    def test_valid_binary_pair_read_only(self):
        before = (self.map_file.read_bytes(), self.image.read_bytes())
        self.assertEqual(validate_saved_map(self.map_file), str(self.map_file.resolve()))
        self.assertEqual(before, (self.map_file.read_bytes(), self.image.read_bytes()))

    def test_binary_pixel_whitespace_is_not_stripped(self):
        self.image.write_bytes(b'P5\r\n2 2\r\n255\r\n\n\r\t ')
        validate_saved_map(self.map_file)

    def test_ascii_pgm_with_comments(self):
        self.image.write_bytes(b'P2\n2 2\n255\n0 254 # free\n205 254\n# end\n')
        validate_saved_map(self.map_file)

    def test_missing_yaml(self):
        with self.assertRaises(FileNotFoundError):
            validate_saved_map(self.root / 'absent.yaml')

    def test_missing_image(self):
        self.metadata['image'] = 'missing.pgm'
        self.write_metadata()
        with self.assertRaises(FileNotFoundError):
            validate_saved_map(self.map_file)

    def test_bad_yaml(self):
        for content in ('[]', 'image: [', ''):
            with self.subTest(content=content):
                self.map_file.write_text(content, encoding='utf-8')
                with self.assertRaises((ValueError, yaml.YAMLError)):
                    validate_saved_map(self.map_file)

    def test_invalid_metadata(self):
        cases = [
            ('image', ''), ('image', 7), ('resolution', 0), ('resolution', -0.05),
            ('resolution', float('nan')), ('resolution', float('inf')),
            ('resolution', True), ('origin', [0, 0]), ('origin', [0, 0, '0']),
            ('origin', [0, 0, float('inf')]), ('free_thresh', 0.9),
            ('occupied_thresh', 2), ('negate', 2), ('negate', True), ('mode', 'typo'),
        ]
        original = dict(self.metadata)
        for key, value in cases:
            with self.subTest(key=key, value=value):
                self.metadata = {**original, key: value}
                self.write_metadata()
                with self.assertRaises(ValueError):
                    validate_saved_map(self.map_file)

    def test_corrupt_pgm(self):
        for data in (
            b'', b'PNG', b'P5\n2 2\n255\n\x00',
            b'P5\n0 2\n255\n', b'P5\n2 2\n65535\n',
            b'P2\n2 2\n255\n0 1 2 256', b'P2\n2 2\n255\n0 1 2',
            b'P2\n2 2\n255\n0 1 2 3 4',
        ):
            with self.subTest(data=data):
                self.image.write_bytes(data)
                with self.assertRaises(ValueError):
                    validate_saved_map(self.map_file)

    def test_missing_map_creates_no_nodes(self):
        context = LaunchContext()
        context.launch_configurations['map_file'] = str(self.root / 'missing.yaml')
        with patch.object(ENTRY, 'Node') as node:
            with self.assertRaises(FileNotFoundError):
                ENTRY._launch_setup(context)
            node.assert_not_called()

    def test_only_four_localization_nodes(self):
        context = LaunchContext()
        context.launch_configurations['map_file'] = str(self.map_file)
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(ENTRY, 'Node', side_effect=lambda **kwargs: kwargs):
            nodes = ENTRY._launch_setup(context)
        self.assertEqual([(n['package'], n['executable']) for n in nodes], [
            ('raicom_control', 'scan_normalizer'), ('nav2_map_server', 'map_server'),
            ('nav2_amcl', 'amcl'), ('nav2_lifecycle_manager', 'lifecycle_manager'),
        ])
        self.assertEqual(nodes[1]['parameters'][1]['yaml_filename'], str(self.map_file))
        self.assertEqual(nodes[3]['parameters'][0]['node_names'], ['map_server', 'amcl'])
        self.assertEqual(nodes[0]['parameters'][0]['output_topic'], '/scan_stable')

    def test_missing_params_creates_no_nodes(self):
        context = LaunchContext()
        context.launch_configurations['map_file'] = str(self.map_file)
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(self.root)), \
                patch.object(ENTRY, 'Node') as node:
            with self.assertRaises(FileNotFoundError):
                ENTRY._launch_setup(context)
            node.assert_not_called()

    def test_existing_normalizer_is_not_started_again(self):
        context = LaunchContext()
        context.launch_configurations.update(
            map_file=str(self.map_file), start_scan_normalizer='false')
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(ENTRY, 'Node', side_effect=lambda **kwargs: kwargs):
            nodes = ENTRY._launch_setup(context)
        self.assertFalse(nodes[0]['condition'].evaluate(context))
        self.assertTrue(all('condition' not in n for n in nodes[1:]))

    def test_default_still_starts_normalizer(self):
        context = LaunchContext()
        description = ENTRY.generate_launch_description()
        for action in description.entities[:2]:
            action.execute(context)
        context.launch_configurations['map_file'] = str(self.map_file)
        with patch.object(ENTRY, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(ENTRY, 'Node', side_effect=lambda **kwargs: kwargs):
            nodes = ENTRY._launch_setup(context)
        self.assertTrue(nodes[0]['condition'].evaluate(context))

    def test_corrupt_image_creates_no_nodes(self):
        self.image.write_bytes(b'P5\n2 2\n255\n\x00')
        context = LaunchContext()
        context.launch_configurations['map_file'] = str(self.map_file)
        with patch.object(ENTRY, 'Node') as node:
            with self.assertRaises(ValueError):
                ENTRY._launch_setup(context)
            node.assert_not_called()

    def test_localization_requires_manual_pose_and_supports_strafing(self):
        params = yaml.safe_load((ROOT / 'config' / 'localization_home.yaml').read_text())
        amcl = params['amcl']['ros__parameters']
        self.assertEqual(amcl['robot_model_type'], 'nav2_amcl::OmniMotionModel')
        self.assertIs(amcl['set_initial_pose'], False)
        self.assertIs(amcl['always_reset_initial_pose'], True)
        self.assertNotIn('initial_pose', amcl)
        self.assertIs(amcl['use_sim_time'], False)
        self.assertEqual(amcl['scan_topic'], '/scan_stable')
        self.assertEqual(set(params), {'amcl', 'map_server'})


if __name__ == '__main__':
    unittest.main()
