"""Exercise the real ROS callbacks with in-memory message/publisher stand-ins."""
import importlib.util
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
    def test_gate_fault_ack_and_rearm(self):
        module = load_monitor()
        now = [0.]
        with patch('time.monotonic', side_effect=lambda: now[0]):
            m = module.MotionConsistencyMonitor()
            ack = False
            x = 0.
            state_history = []
            for i in range(321):
                t = now[0] = i*0.05
                # Warm up, drive normally, stall, keep commanding into fault,
                # then let autonomy cease commanding and re-arm.
                command = 0.4 if 5 <= t < 11 else 0.
                vx = command if not ack else 0.
                if 5 <= t < 7: x += vx*0.05
                m.on_command(NS(linear=NS(x=command,y=0.), angular=NS(z=0.)))
                m.on_ack(Message(data=ack))
                m.on_gyro(Message(data='OK'))
                m.on_drive(odom(t, x, vx))
                if i % 2 == 0: m.on_rf(odom(t, x, 0.))
                m.evaluate()
                if m.request_pub.publish.called:
                    ack = m.request_pub.publish.call_args.args[0].data
                    m.request_pub.publish.reset_mock()
                state_history.append((t,m.recovery.state,m.gate))
                if 9 <= t < 11:
                    self.assertFalse(m.gate)
                    self.assertTrue(ack)
            self.assertTrue(any(5<t<7 and gate for t,_,gate in state_history))
            self.assertTrue(any(state=='STALLED' for _,state,_ in state_history))
            self.assertEqual(m.recovery.state, 'NORMAL')
            self.assertFalse(ack)
            self.assertTrue(m.gate)
            self.assertGreater(m.drive_pub.publish.call_count, 0)
            self.assertGreater(m.rf_pub.publish.call_args.args[0].pose.covariance[0], 0)
            # Loss of RF2O closes the gate and requests a stop.
            now[0] = 20.
            m.evaluate()
            self.assertFalse(m.gate)
            self.assertTrue(m.request_pub.publish.call_args.args[0].data)
            old_rf_time = m.rf_at
            m.on_rf(odom(10., 0., 0.))
            self.assertEqual(m.rf_at, old_rf_time)
