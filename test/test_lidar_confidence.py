"""Exercise confidence callbacks, TF use and missing-input behavior without DDS."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import Mock, patch

import numpy as np


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Node:
    def __init__(self, *args): pass
    def declare_parameter(self, name, default): return NS(value=default)
    def create_subscription(self, *args): pass
    def create_timer(self, *args): pass
    def create_publisher(self, *args): return Mock()
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=int(self.test_time*1e9)))


def load_node():
    modules = {}
    for name, attrs in {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'rclpy.qos': {'qos_profile_sensor_data': None},
        'rclpy.time': {'Time': NS(from_msg=lambda value: value)},
        'nav_msgs.msg': {'Odometry': Message}, 'sensor_msgs.msg': {'LaserScan': Message, 'Imu': Message},
        'std_msgs.msg': {'String': Message}, 'diagnostic_msgs.msg': {'DiagnosticArray': Message},
        'tf2_ros': {'Buffer': Mock, 'TransformListener': Mock, 'TransformException': RuntimeError},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        modules[name] = module
    spec = importlib.util.spec_from_file_location(
        'armatron._confidence_tested', Path(__file__).parents[1]/'armatron/lidar_confidence.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def header(t, frame='base_link'):
    ns = round(t*1e9)
    sec, nanosec = divmod(ns, 1000000000)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame)


class ConfidenceNodeTest(unittest.TestCase):
    def test_reversed_mount_and_tf_failure(self):
        node = load_node().LidarConfidence()
        node.tf.lookup_transform.return_value = NS(transform=NS(
            rotation=NS(x=0., y=0., z=1., w=0.), translation=NS(x=0.125, y=0., z=0.)))
        scan = NS(header=header(1., 'laser'), ranges=[1., 2., float('inf')],
                  angle_min=0., angle_increment=math.pi/2, range_min=0.1, range_max=10.)
        node.on_scan(scan)
        np.testing.assert_allclose(node.scans[-1][1][:2], [[-0.875, 0.], [0.125, -2.]], atol=1e-10)
        self.assertTrue(np.isnan(node.scans[-1][1][2]).all())
        node.tf.lookup_transform.side_effect = RuntimeError('missing')
        scan.header = header(2., 'laser')
        node.on_scan(scan)
        self.assertEqual(len(node.scans), 1)

    def test_pipeline_and_no_fallback_to_old_native_metrics(self):
        module = load_node()
        node = module.LidarConfidence()
        # Dense rectangle perimeter with valid local surface normals.
        line = np.linspace(-2., 2., 120)
        points = np.vstack((np.column_stack((line, np.full(120, -2.))),
                            np.column_stack((np.full(120, 2.), line)),
                            np.column_stack((line[::-1], np.full(120, 2.))),
                            np.column_stack((np.full(120, -2.), line[::-1]))))
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            for i in range(31):
                t = now[0] = node.test_time = i*0.1
                node.drive.add(t, (0., 0., 0.))
                node.gyro.add(t, (0., 0., 0.))
                node.rf.add(t, (0., 0., 0.))
                node.scans.append((t, points))
                node.on_solver(NS(header=header(t, 'laser'), status=[NS(name='rf2o_solver',
                                              values=[NS(key='valid', value='1')])]))
                node.evaluate()
            result = json.loads(node.publisher.publish.call_args.args[0].data)
            self.assertEqual(result['state'], 'CONSISTENT', result)
            self.assertEqual(result['frame_id'], 'base_link')
            node.solver.clear()
            node.solver.append((3.05, {'valid': '1'}))
            now[0] = node.test_time = 3.6
            for history in (node.drive, node.gyro, node.rf):
                for i in range(31, 37): history.add(i*0.1, (0., 0., 0.))
            node.scans.append((3.6, points))
            node.evaluate()
            result = json.loads(node.publisher.publish.call_args.args[0].data)
            self.assertEqual(result['state'], 'UNAVAILABLE')
            self.assertIn('diagnostics missing', result['reason'])
