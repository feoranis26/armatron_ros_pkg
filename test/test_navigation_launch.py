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
