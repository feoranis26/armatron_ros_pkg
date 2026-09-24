"""Exercise launch construction without starting ROS processes."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch
from armatron.map_state import save_state


class Item:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


def load_launch():
    modules = {}
    for name, attrs in {
        'launch': {'LaunchDescription': list},
        'launch.actions': {'DeclareLaunchArgument': Item, 'IncludeLaunchDescription': Item},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': Item},
        'launch.substitutions': {'LaunchConfiguration': Item, 'PythonExpression': Item,
                                 'PathJoinSubstitution': lambda parts: '/'.join(parts)},
        'launch_ros.substitutions': {'FindPackageShare': lambda name: '/share/'+name},
        'launch_ros.actions': {'Node': Item},
        'armatron.launch_parameters': {'RewrittenYaml': Item},
    }.items():
        modules[name] = ModuleType(name)
        modules[name].__dict__.update(attrs)
    path = Path(__file__).parents[1] / 'launch/navigation.launch.py'
    spec = importlib.util.spec_from_file_location('_navigation_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class NavigationLaunchTest(unittest.TestCase):
    def test_amcl_requires_grid_and_disables_slam(self):
        module = load_launch()
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'ARMATRON_STATE_DIR': directory}):
            root = Path(directory)
            save_state({'active_profile': 'test', 'mode': 'amcl',
                        'last_pose': {'x': 2., 'y': -1., 'yaw': .5}}, root)
            with self.assertRaisesRegex(RuntimeError, 'grid export'):
                module.generate_launch_description()
            grid = root / 'maps/test/current/grid'
            grid.mkdir(parents=True)
            (grid/'map.yaml').write_text('image: map.pgm')
            actions = module.generate_launch_description()
            include = next(a for a in actions if 'launch_arguments' in a.kwargs)
            arguments = dict(include.kwargs['launch_arguments'])
            self.assertEqual(arguments['slam'], 'False')
            rewrites = arguments['params_file'].kwargs['param_rewrites']
            self.assertEqual(rewrites['amcl.ros__parameters.initial_pose.x'], '2.0')
            pose_node = next(a for a in actions if a.kwargs.get('executable') == 'pose_persistence')
            self.assertEqual(pose_node.kwargs['parameters'], [{'pose_topic':'/amcl_pose'}])

    def test_mapping_still_uses_slam_pose_and_does_not_require_grid(self):
        module = load_launch()
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'ARMATRON_STATE_DIR': directory}):
            save_state({'active_profile': 'test', 'mode': 'mapping'}, Path(directory))
            actions = module.generate_launch_description()
            pose_node = next(a for a in actions if a.kwargs.get('executable') == 'pose_persistence')
            self.assertEqual(pose_node.kwargs['parameters'], [{'pose_topic':'/pose'}])

    def test_localization_owns_one_localizer_and_starts_navigation_only(self):
        module = load_launch()
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'ARMATRON_STATE_DIR': directory}):
            root = Path(directory)
            save_state({'active_profile': 'test', 'mode': 'localization',
                        'last_pose': {'x': 2., 'y': -1., 'yaw': .5}}, root)
            with self.assertRaisesRegex(RuntimeError, 'no saved posegraph'):
                module.generate_launch_description()
            revision = root/'maps/test/current'
            revision.mkdir(parents=True)
            (revision/'map.posegraph').write_bytes(b'graph')
            with self.assertRaisesRegex(RuntimeError, 'saved grid'):
                module.generate_launch_description()
            (revision/'grid').mkdir()
            (revision/'grid/map.yaml').write_text('image: map.pgm')
            actions = module.generate_launch_description()
            localizers = [a for a in actions if a.kwargs.get('package') == 'slam_toolbox']
            self.assertEqual(len(localizers), 1)
            self.assertEqual(localizers[0].kwargs['executable'], 'localization_slam_toolbox_node')
            settings = localizers[0].kwargs['parameters'][1]
            self.assertFalse(settings['enable_interactive_mode'])
            self.assertEqual(settings['map_name'], '/slam_toolbox/localization_map')
            self.assertEqual(settings['scan_buffer_size'], 3)
            maps = [a for a in actions if a.kwargs.get('executable') == 'map_server']
            self.assertEqual(len(maps), 1)
            self.assertEqual(maps[0].kwargs['parameters'][0]['yaml_filename'], str(revision/'grid/map.yaml'))
            manager = next(a for a in actions if a.kwargs.get('name') == 'lifecycle_manager_map')
            self.assertEqual(manager.kwargs['parameters'][0]['node_names'], ['map_server'])
            includes = [a for a in actions if 'launch_arguments' in a.kwargs]
            self.assertEqual(len(includes), 1)
            self.assertTrue(includes[0].args[0].args[0].endswith('/navigation_launch.py'))
            arguments = dict(includes[0].kwargs['launch_arguments'])
            self.assertNotIn('slam', arguments)
            self.assertNotIn('map', arguments)
            self.assertEqual(arguments['use_composition'], 'False')
            self.assertIs(localizers[0].kwargs['parameters'][0], arguments['params_file'])
            rewrites = arguments['params_file'].kwargs['param_rewrites']
            self.assertEqual(rewrites['slam_toolbox.ros__parameters.map_start_pose.0'], '2.0')
