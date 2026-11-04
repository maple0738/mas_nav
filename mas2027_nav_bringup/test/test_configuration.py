import importlib.util
from pathlib import Path
import tempfile
import shutil
import subprocess
import sys
import unittest

import yaml

SOURCE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('navigation_configuration', SOURCE / 'mas2027_nav_bringup/configuration.py')
configuration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(configuration)


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def manifest(self, **values):
        path = self.root / 'profile.yaml'
        path.write_text(yaml.safe_dump({'map_source': 'online', **values}))
        return path

    def bundle(self):
        maps = self.root / 'maps with spaces'
        maps.mkdir()
        for name in ['cloud.pcd', 'terrain.msgpack', 'floor.pgm']:
            (maps / name).write_bytes(b'test')
        (maps / 'floor.yaml').write_text('image: floor.pgm\n')
        files = dict(localization_pcd='cloud.pcd', occupancy_yaml='floor.yaml', terrain_msgpack='terrain.msgpack')
        return maps, files

    def test_online_needs_no_map_assets(self):
        manifest = configuration.read_manifest(self.manifest())
        self.assertEqual(manifest['map_source'], 'online')
        self.assertEqual(manifest['map_files'], {})

    def test_prior_resolves_paths_relative_to_manifest_and_yaml(self):
        maps, files = self.bundle()
        manifest = configuration.read_manifest(self.manifest(map_source='prior', map_root=maps.name, map_files=files))
        resolved = configuration.resolve_map_files(manifest)
        self.assertEqual(resolved['occupancy_image'], str(maps / 'floor.pgm'))
        self.assertEqual(resolved['localization_pcd'], str(maps / 'cloud.pcd'))

    def test_absolute_map_root_and_mode_override(self):
        maps, files = self.bundle()
        manifest = configuration.read_manifest(self.manifest(map_root=str(maps), map_files=files), 'prior')
        self.assertEqual(manifest['map_source'], 'prior')
        self.assertEqual(configuration.resolve_map_files(manifest)['terrain_msgpack'], str(maps / 'terrain.msgpack'))

    def test_missing_image_is_rejected(self):
        maps, files = self.bundle()
        (maps / 'floor.pgm').unlink()
        manifest = configuration.read_manifest(self.manifest(map_source='prior', map_root=str(maps), map_files=files))
        with self.assertRaisesRegex(ValueError, 'image does not exist'):
            configuration.resolve_map_files(manifest)

    def test_unknown_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'online or prior'):
            configuration.read_manifest(self.manifest(map_source='wrong'))

    def test_parameter_files_have_no_duplicate_keys(self):
        paths = configuration.parameter_files(SOURCE.parent / 'mas2027_nav_executor/config')
        seen = set()
        def flatten(value, prefix=''):
            for key, child in value.items():
                name = f'{prefix}.{key}' if prefix else key
                if isinstance(child, dict):
                    yield from flatten(child, name)
                else:
                    yield name
        for path in paths:
            document = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
            for key in flatten(document):
                self.assertNotIn(key, seen, f'{path}: duplicate {key}')
                seen.add(key)
        self.assertNotIn('node.map_source', seen)
        self.assertIn('planner.rog_map.projection.terrain.tunnel.min_neighbor_support', seen)

    def test_missing_parameter_file_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Missing navigation parameter file'):
            configuration.parameter_files(self.root)

    def test_duplicate_parameter_is_rejected_at_startup(self):
        original = SOURCE.parent / 'mas2027_nav_executor/config'
        for name in configuration.PARAMETER_FILES:
            shutil.copy(original / name, self.root / name)
        path = self.root / 'mpc_params.yaml'
        document = yaml.safe_load(path.read_text())
        document['/**']['ros__parameters']['node'] = {'control_rate_hz': 10.0}
        path.write_text(yaml.safe_dump(document))
        with self.assertRaisesRegex(ValueError, 'duplicate parameter node.control_rate_hz'):
            configuration.parameter_files(self.root)

    def test_online_packaging_allows_missing_maps_but_prior_rejects_them(self):
        script = SOURCE / 'mas2027_nav_bringup/configuration.py'
        path = self.manifest()
        online = subprocess.run([sys.executable, str(script), str(path)], capture_output=True)
        self.assertEqual(online.returncode, 0, online.stderr)
        self.assertEqual(online.stdout, b'')
        path = self.manifest(map_source='prior')
        prior = subprocess.run([sys.executable, str(script), str(path)], capture_output=True)
        self.assertNotEqual(prior.returncode, 0)

    def test_default_prior_bundle_is_complete(self):
        manifest = configuration.read_manifest(SOURCE / 'config/navigation_map.yaml', 'prior')
        self.assertEqual(len(configuration.resolve_map_files(manifest)), 4)


if __name__ == '__main__':
    unittest.main()
