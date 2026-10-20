import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'deploy'))
from map_localization_recovery import route_point_prior, route_start_prior


class RouteStartPriorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.yaml = root / 'map.yaml'
        self.image = root / 'map.pgm'
        self.yaml.write_bytes(b'yaml')
        self.image.write_bytes(b'pgm')
        self.binding = {
            'width': 178, 'height': 162,
            'resolution': 0.019999999552965164,
            'cells_sha256': 'live-cell-digest',
        }
        self.route = {
            'schema': 'raicom_manual_route/v1', 'frame': 'map',
            'map': {
                'saved': {
                    'yaml_sha256': hashlib.sha256(b'yaml').hexdigest(),
                    'image_sha256': hashlib.sha256(b'pgm').hexdigest(),
                },
                'loaded': {'width': 178, 'height': 162, 'resolution': 0.02,
                           'cells_sha256': 'documented-different-encoding'},
            },
            'points': {
                'start': {'x': .1, 'y': -.2, 'yaw_rad': .3},
                'park_2': {'x': 1.03, 'y': .59, 'yaw_rad': 1.59},
            },
        }

    def tearDown(self):
        self.temp.cleanup()

    def make(self):
        return route_start_prior(self.route, self.binding,
                                 yaml_path=self.yaml, image_path=self.image)

    def test_exact_assets_and_geometry_build_live_bound_prior(self):
        prior = self.make()
        self.assertEqual(prior['map'], self.binding)
        self.assertEqual(prior['position'], [.1, -.2, 0.0])
        self.assertEqual(prior['route_point'], 'start')
        self.assertAlmostEqual(prior['quaternion'][2], math.sin(.15))
        self.assertEqual(prior['covariance'][0], .03 ** 2)
        self.assertEqual(prior['covariance'][35], .05 ** 2)

    def test_asset_hash_change_is_rejected(self):
        self.image.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'asset hash'):
            self.make()

    def test_geometry_and_invalid_start_are_rejected(self):
        self.route['map']['loaded']['width'] = 177
        with self.assertRaisesRegex(ValueError, 'geometry'):
            self.make()
        self.route['map']['loaded']['width'] = 178
        self.route['points']['start']['yaw_rad'] = math.nan
        with self.assertRaisesRegex(ValueError, 'pose'):
            self.make()

    def test_named_route_point_is_selected_explicitly(self):
        prior = route_point_prior(
            self.route, self.binding, 'park_2',
            yaml_path=self.yaml, image_path=self.image)
        self.assertEqual(prior['route_point'], 'park_2')
        self.assertEqual(prior['position'], [1.03, .59, 0.0])
        self.assertAlmostEqual(prior['quaternion'][2], math.sin(1.59 / 2))

    def test_missing_named_route_point_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'park_1'):
            route_point_prior(
                self.route, self.binding, 'park_1',
                yaml_path=self.yaml, image_path=self.image)


if __name__ == '__main__':
    unittest.main()
