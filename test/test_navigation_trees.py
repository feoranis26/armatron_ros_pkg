"""Check both navigator defaults against the configured BT plugin set."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


class NavigationTreesTest(unittest.TestCase):
    def test_both_defaults_are_installed_no_backup_trees_with_available_plugins(self):
        root = Path(__file__).parents[1]
        launch = ast.parse((root / 'launch/navigation.launch.py').read_text())
        rewrites = next(n.value for n in ast.walk(launch)
                        if isinstance(n, ast.keyword) and n.arg == 'param_rewrites')
        params = dict(zip((k.value for k in rewrites.keys), rewrites.values))
        navigator = yaml.safe_load((root / 'config/nav2/navigation.yaml').read_text())[
            'bt_navigator']['ros__parameters']
        plugins = navigator['plugin_lib_names']
        required = {
            'ComputePathToPose': 'nav2_compute_path_to_pose_action_bt_node',
            'ComputePathThroughPoses': 'nav2_compute_path_through_poses_action_bt_node',
            'RemovePassedGoals': 'nav2_remove_passed_goals_action_bt_node',
            'FollowPath': 'nav2_follow_path_action_bt_node',
            'ClearEntireCostmap': 'nav2_clear_costmap_service_bt_node',
            'Spin': 'nav2_spin_action_bt_node', 'Wait': 'nav2_wait_action_bt_node',
            'GoalUpdated': 'nav2_goal_updated_condition_bt_node',
            'RecoveryNode': 'nav2_recovery_node_bt_node',
            'PipelineSequence': 'nav2_pipeline_sequence_bt_node',
            'RateController': 'nav2_rate_controller_bt_node',
        }
        # BehaviorTree.CPP 3.8 registers the memory sequence as SequenceStar.
        # Source: BehaviorTree/BehaviorTree.CPP tag 3.8.6, src/bt_factory.cpp.
        builtins = {'root', 'BehaviorTree', 'ReactiveFallback', 'ReactiveSequence',
                    'SequenceStar'}
        for kind in ('to_pose', 'through_poses'):
            # Humble RewrittenYaml silently ignores absent parameter keys.
            key = f'default_nav_{kind}_bt_xml'
            self.assertIn(key, navigator)
            self.assertIsInstance(navigator[key], str)
            expression = params[f'bt_navigator.ros__parameters.default_nav_{kind}_bt_xml']
            filename = next(n.value for n in ast.walk(expression)
                            if isinstance(n, ast.Constant) and isinstance(n.value, str)
                            and n.value.endswith('.xml'))
            tree = ET.parse(root / 'behavior_trees' / filename)
            self.assertNotIn('BTCPP_format', tree.getroot().attrib)
            tags = {node.tag for node in tree.iter()}
            self.assertNotIn('SequenceWithMemory', tags)
            self.assertNotIn('BackUp', tags)
            for tag in tags - builtins:
                self.assertIn(required[tag], plugins)
            planner = 'ComputePathToPose' if kind == 'to_pose' else 'ComputePathThroughPoses'
            self.assertIn(planner, tags)
            if kind == 'through_poses':
                self.assertEqual(tree.find('.//ComputePathThroughPoses').get('goals'), '{goals}')
