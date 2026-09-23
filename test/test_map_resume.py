import ast
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch
import yaml

from armatron.map_manager import set_selection
from armatron.map_state import load_state, save_state


class MapResumeTest(unittest.TestCase):
    def test_pose_capture_uses_actual_topic_and_rejects_wrong_frame(self):
        modules = {}
        class Node:
            def __init__(self, *args):
                self.create_subscription = Mock()
                self.create_timer = Mock()
        for name, attrs in {'rclpy': {}, 'rclpy.node': {'Node': Node},
                            'geometry_msgs': {}, 'geometry_msgs.msg':
                            {'PoseWithCovarianceStamped': object}}.items():
            modules[name] = ModuleType(name)
            modules[name].__dict__.update(attrs)
        path = Path(__file__).parents[1] / 'armatron/pose_persistence.py'
        spec = importlib.util.spec_from_file_location('armatron._pose_test', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        state = {'active_profile': 'one', 'mode': 'mapping'}
        with patch.object(module, 'load_state', side_effect=lambda: dict(state)), \
                patch.object(module, 'save_state') as save:
            node = module.PosePersistence()
            self.assertEqual(node.create_subscription.call_args.args[1], '/pose')
            message = SimpleNamespace(header=SimpleNamespace(frame_id='odom'),
                pose=SimpleNamespace(pose=SimpleNamespace(
                    position=SimpleNamespace(x=5., y=-2.),
                    orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
            node.on_pose(message)
            save.assert_not_called()
            message.header.frame_id = 'map'
            node.on_pose(message)
            self.assertEqual(save.call_args.args[0]['last_pose'], {'x': 5., 'y': -2., 'yaw': 0.})
            save.reset_mock()
            state['active_profile'] = 'two'
            node.persist()
            save.assert_not_called()

    def test_mode_change_preserves_pose_but_profile_change_does_not(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ('one', 'two'):
                (root / 'maps' / name).mkdir(parents=True)
            pose = {'x': 5.2, 'y': -3.1, 'yaw': 1.2}
            save_state({'active_profile': 'one', 'mode': 'mapping', 'last_pose': pose}, root)
            set_selection('one', 'localization', root)
            self.assertEqual(load_state(root)['last_pose'], pose)
            set_selection('two', 'mapping', root)
            self.assertIsNone(load_state(root)['last_pose'])

    def test_resume_array_rewrites_address_existing_humble_leaf_paths(self):
        root = Path(__file__).parents[1]
        tree = ast.parse((root / 'launch/navigation.launch.py').read_text())
        rewrites = next(n.value for n in ast.walk(tree)
                        if isinstance(n, ast.keyword) and n.arg == 'param_rewrites')
        expression = next(v for k,v in zip(rewrites.keys, rewrites.values) if k is None)
        pose = [5.2, -3.1, 1.2]
        changes = eval(compile(ast.Expression(expression), '<restart pose>', 'eval'),
                       {'map_start_pose': pose})
        params = yaml.safe_load((root / 'config/nav2/navigation.yaml').read_text())
        # Load the production path updater without requiring a ROS installation.
        stub = ModuleType('nav2_common.launch')
        stub.RewrittenYaml = object
        spec = importlib.util.spec_from_file_location('armatron._launch_params_test',
            root / 'armatron/launch_parameters.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'nav2_common': ModuleType('nav2_common'),
                                     'nav2_common.launch': stub}):
            spec.loader.exec_module(module)
        updater = module.RewrittenYaml()
        # Humble pathify expands lists to .0/.1/.2. Whole-array rewrites
        # are ignored. Verify these exact paths exist and preserve double type.
        for path, value in changes.items():
            keys = path.split('.')
            target = params
            for key in keys[:-1]:
                target = target[key]
            index = int(keys[-1])
            self.assertIsInstance(target[index], float)
            updater.updateYamlPathVals(params, keys, float(value))
        self.assertEqual(params['slam_toolbox']['ros__parameters']['map_start_pose'], pose)
        explicit = {k.value for k in rewrites.keys if k is not None}
        for name in ('max_velocity', 'min_velocity'):
            self.assertIn(f'velocity_smoother.ros__parameters.{name}.1', explicit)
            self.assertNotIn(f'velocity_smoother.ros__parameters.{name}', explicit)
            updater.updateYamlPathVals(params,
                f'velocity_smoother.ros__parameters.{name}.1'.split('.'), 0.0)
            self.assertEqual(params['velocity_smoother']['ros__parameters'][name][1], 0.0)
        updater.updateYamlPathVals(params,
            ['slam_toolbox', 'ros__parameters', 'map_file_name'], '/tmp/map')
        self.assertEqual(params['slam_toolbox']['ros__parameters']['map_file_name'], '/tmp/map')
