"""Check launch profiles without starting ROS processes or hardware."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE))
spec = importlib.util.spec_from_file_location('navigation_launch', SOURCE / 'launch/nav_executor_launch.py')
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)


class CaptureNode:
    def __init__(self, **kwargs):
        self.values = kwargs


class NavigationLaunchTest(unittest.TestCase):
    def launch(self, mode):
        context = LaunchContext()
        context.launch_configurations.update({
            'navigation_config': str(SOURCE / 'config/navigation_map.yaml'),
            'config_dir': str(SOURCE.parent / 'mas2027_nav_executor/config'),
            'map_source': mode, 'output_topic': '', 'normal_mode': '',
            'use_foxglove': 'false',
        })
        original = launch_module.get_package_share_directory
        def package(name):
            if name == 'mas2027_nav_bringup':
                return str(SOURCE)
            if name == 'odom_localizer':
                return str(SOURCE.parent / 'mas2027_perception/Localization/odom_localizer')
            return original(name)
        with tempfile.TemporaryDirectory() as temporary:
            read = launch_module.read_manifest
            def manifest(*args):
                result = read(*args)
                result['log_dir'] = temporary
                return result
            with patch.object(launch_module, 'Node', CaptureNode), patch.object(launch_module, 'read_manifest', manifest), patch.object(launch_module, 'get_package_share_directory', package):
                actions = launch_module._launch_setup(context)
        return [action.values for action in actions if isinstance(action, CaptureNode)]

    def test_online_has_no_static_map_or_localizer(self):
        nodes = self.launch('online')
        packages = {node['package'] for node in nodes}
        self.assertNotIn('map_server', packages)
        self.assertNotIn('odom_localizer', packages)
        executor = next(node for node in nodes if node['package'] == 'mas2027_nav_executor')
        parameters = executor['parameters']
        self.assertEqual(len([value for value in parameters if isinstance(value, str)]), 6)
        overrides = {key: value for item in parameters if isinstance(item, dict) for key, value in item.items()}
        self.assertEqual(overrides['node.map_source'], 'online')
        self.assertTrue(overrides['planner.rog_map.projection.unknown_as_occupied'])
        self.assertNotIn('region_control.normal_mode', overrides)
        self.assertNotIn('node.topics.cmd_vel_pub', overrides)

    def test_prior_has_matching_map_bundle(self):
        nodes = self.launch('prior')
        packages = {node['package'] for node in nodes}
        self.assertIn('map_server', packages)
        self.assertIn('odom_localizer', packages)
        server = next(node for node in nodes if node['package'] == 'map_server')['parameters'][0]
        executor = next(node for node in nodes if node['package'] == 'mas2027_nav_executor')
        overrides = {key: value for item in executor['parameters'] if isinstance(item, dict) for key, value in item.items()}
        self.assertEqual(overrides['node.map_source'], 'prior')
        self.assertFalse(any(key.startswith('planner.rog_map.projection.prior_map') for key in overrides))
        self.assertTrue(Path(server['map_yaml_path']).is_file())
        self.assertTrue(Path(server['terrain_map_path']).is_file())


if __name__ == '__main__':
    unittest.main()
