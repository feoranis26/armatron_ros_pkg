"""Exercise the real ROS callbacks with in-memory message/publisher stand-ins."""
import importlib.util
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import Mock, patch


class Message:
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


class Node:
    def __init__(self, *args): pass
    def declare_parameter(self, name, default): return NS(value=default)
    def create_subscription(self, *args): pass
    def create_timer(self, *args): pass
    def create_service(self, *args): pass
    def create_publisher(self, kind, topic, depth):
        return NS(publish=Mock())
    def get_logger(self): return Mock()
    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(time.monotonic()*1e9)))


def load_monitor():
    modules = {}
    for name, attrs in {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'geometry_msgs.msg': {'Twist': Message},
        'nav_msgs.msg': {'Odometry': Message},
        'std_msgs.msg': {'Bool': Message, 'String': Message},
        'std_srvs.srv': {'Empty': Message},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        modules[name] = module
    name = 'armatron._monitor_under_test'
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).parents[1] / 'armatron/motion_consistency.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def odom(t, x, vx):
    return NS(header=NS(stamp=NS(sec=int(t), nanosec=int((t-int(t))*1e9))),
              pose=NS(pose=NS(position=NS(x=x,y=0.), orientation=NS(x=0.,y=0.,z=0.,w=1.)),
                      covariance=[0.]*36),
              twist=NS(twist=NS(linear=NS(x=vx,y=0.), angular=NS(z=0.)), covariance=[0.]*36))


class MonitorTest(unittest.TestCase):
    def feed(self, m, t, speed, x, ack=False):
        m.on_ack(Message(data=ack))
        m.on_gyro(Message(data='OK'))
        m.on_drive(odom(t, 0., speed))
        m.on_rf(odom(t, x, 0.))
        m.evaluate()

    def test_default_disagreement_and_sensor_loss_are_diagnostic_only(self):
        module = load_monitor()
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            m = module.MotionConsistencyMonitor()
            for i in range(101):
                now[0] = i*0.1
                self.feed(m, now[0], 0.4, 0.)
            self.assertEqual(m.last_status, 'DISAGREEMENT')
            self.assertFalse(m.gate)
            m.drive_pub.publish.assert_not_called()
            self.assertGreater(m.rf_pub.publish.call_count, 0)
            self.assertFalse(hasattr(m, 'request_pub'))
            now[0] = 12.
            m.evaluate()
            self.assertEqual(m.last_status, 'UNAVAILABLE')
            # An existing explicit stop is observed, never automatically reset.
            self.feed(m, now[0], 0., 0., ack=True)
            self.assertTrue(m.ack)
            self.assertFalse(hasattr(m, 'recovery'))

    def test_optional_wheel_fusion_waits_for_sustained_agreement(self):
        module = load_monitor()
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            m = module.MotionConsistencyMonitor()
            m.use_drive = True
            x = 0.
            for i in range(161):
                t = now[0] = i*0.1
                # Healthy motion, then disagreement, then healthy motion again.
                if t < 5 or t >= 8:
                    x += 0.04
                self.feed(m, t, 0.4, x)
                if 4.5 < t < 5:
                    self.assertTrue(m.gate)
                if 6 < t < 11:
                    self.assertFalse(m.gate)
            self.assertTrue(m.gate)
            self.assertGreater(m.drive_pub.publish.call_count, 0)
            now[0] = 17.
            m.evaluate()
            self.assertFalse(m.gate)
