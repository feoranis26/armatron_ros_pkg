"""Exercise the real ROS callbacks with in-memory message/publisher stand-ins."""
import importlib.util
import math
import json
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
    return NS(child_frame_id="base_link", header=NS(frame_id="odom", stamp=NS(sec=int(t), nanosec=int((t-int(t))*1e9))),
              pose=NS(pose=NS(position=NS(x=x,y=0.), orientation=NS(x=0.,y=0.,z=0.,w=1.)),
                      covariance=[0.]*36),
              twist=NS(twist=NS(linear=NS(x=vx,y=0.), angular=NS(z=0.)), covariance=[0.]*36))


class MonitorTest(unittest.TestCase):
    def test_carried_motion_publishes_lidar_velocity_without_wheel_input(self):
        module = load_monitor()
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            m = module.MotionConsistencyMonitor()
            for i in range(31):
                t = now[0] = i*0.1
                m.on_confidence(Message(data=json.dumps(dict(schema=2, stamp=t, frame_id='base_link',
                    state='MOTION_CONTRADICTED', candidate_state='MOTION_CONTRADICTED',
                    fusion_state='MOTION_CONTRADICTED', fusion_reference_yaw=0.,
                    fusion_information=[[.5,0.,0.],[0.,.5,0.],[0.,0.,1.]]))))
                m.on_ack(Message(data=False))
                m.on_gyro(Message(data='OK'))
                m.on_drive(odom(t, 0., 0.))
                m.on_rf(odom(t, .2*t, 0.))
            message = m.rf_pub.publish.call_args.args[0]
            self.assertAlmostEqual(message.twist.twist.linear.x, .2)
            self.assertAlmostEqual(message.twist.covariance[0], .0025)
            m.drive_pub.publish.assert_not_called()
            self.assertEqual(m.fusion_mode, 'LIDAR')

    def feed(self, m, t, speed, x, ack=False):
        m.on_confidence(Message(data=json.dumps({'schema': 1, 'stamp': t,
                        'state': 'CONSISTENT', 'candidate_state': 'CONSISTENT'})))
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
            # Raw disagreement alone is not a stall classification.
            self.assertEqual(m.last_status, 'CONSISTENT')
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

    def test_evidence_states_staleness_and_invalid_messages(self):
        module = load_monitor()
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            m = module.MotionConsistencyMonitor()
            for i in range(31):
                now[0] = i*0.1
                self.feed(m, now[0], 0., 0.)
            for state in ('LIDAR_UNDERCONSTRAINED', 'MOTION_CONTRADICTED', 'TRACKING_UNRELIABLE'):
                m.on_confidence(Message(data=json.dumps({'schema': 1, 'stamp': now[0],
                                    'state': state, 'candidate_state': state})))
                m.evaluate()
                self.assertEqual(m.last_status, state)
                self.assertFalse(m.gate)
            for value in ('null', 'not json', '{"stamp": 0}', '[]'):
                m.on_confidence(Message(data=value))
            self.assertFalse(hasattr(m, 'request_pub'))
            now[0] = 4.
            m.evaluate()
            self.assertEqual(m.last_status, 'UNAVAILABLE')

    def test_directional_fallback_and_immediate_contradiction_veto(self):
        module = load_monitor()
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            m = module.MotionConsistencyMonitor()
            for i in range(40):
                t = now[0] = i*0.1
                m.on_confidence(Message(data=json.dumps(dict(schema=2, stamp=t, frame_id='base_link',
                    state='LIDAR_UNDERCONSTRAINED', candidate_state='LIDAR_UNDERCONSTRAINED',
                    fusion_state='LIDAR_UNDERCONSTRAINED', fusion_reference_yaw=0.,
                    fusion_information=[[.02,0.,0.],[0.,.98,0.],[0.,0.,1.]]))))
                m.on_ack(Message(data=False))
                m.on_gyro(Message(data='OK'))
                m.on_drive(odom(t, 0., .2))
                m.on_rf(odom(t, 0., 0.))
            self.assertTrue(m.gate)
            wheel = m.drive_pub.publish.call_args.args[0].twist.covariance
            lidar = m.rf_pub.publish.call_args.args[0].twist.covariance
            self.assertLess(wheel[0], lidar[0])
            self.assertGreater(wheel[7], lidar[7])
            m.confidence['candidate_state'] = 'MOTION_CONTRADICTED'
            m.evaluate()
            self.assertFalse(m.gate)
            self.assertFalse(hasattr(m, 'request_pub'))
            now[0] = 6.
            m.evaluate()
            self.assertFalse(m.gate)
