"""Propulsion remains independent of the diagnostic monitor."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import Mock, patch


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Twist:
    def __init__(self):
        self.linear = NS(x=0., y=0., z=0.)
        self.angular = NS(x=0., y=0., z=0.)


def load_bridge():
    modules = {}
    for name, attrs in {
        'rclpy': {}, 'rclpy.node': {'Node': object},
        'geometry_msgs.msg': {'Twist': Twist, 'Point': Message, 'Quaternion': Message},
        'nav_msgs.msg': {'Odometry': Message}, 'sensor_msgs.msg': {'Imu': Message},
        'std_msgs.msg': {'Bool': Message, 'String': Message, 'Float64': Message},
        'std_srvs.srv': {'Empty': Message},
        'armatron.drive_protocol': {'WheelDriver': Mock()},
        'armatron.gyro_protocol': {'UDPGyro': Mock()},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        modules[name] = module
    spec = importlib.util.spec_from_file_location(
        'armatron._bridge_tested', Path(__file__).parents[1] / 'armatron/drive_bridge.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class DriveSafetyTest(unittest.TestCase):
    def test_no_monitor_needed_but_explicit_stop_and_timeouts_work(self):
        module = load_bridge()
        node = module.ArmatronDrive.__new__(module.ArmatronDrive)
        node.inhibit_requested = None
        node.last_safety_send = -float('inf')
        node.last_safety_sample = None
        node.driver = Mock(safety_sample=(False, 100.), position=[0., 0.], speed=[0., 0.])
        node.safety_publisher = Mock()
        node.gyro = NS(angle=None, sample=None)
        node.gyro_imu = Mock()
        node.gyro_imu.message.return_value = None
        node.get_clock = Mock(return_value=Mock())
        node.get_logger = Mock(return_value=Mock())
        node.heading = node.hold = 0.
        node.base_frame_id = 'base_link'
        node.absolute = False
        node.tgt_speed = [0.4, 0.2, 0.1]
        node.lastSpeedReceived = 100.
        node.last_position = node.odom_speed = [0., 0.]
        node.position = NS(x=0., y=0.)
        node.odom_update = Mock()
        with patch('time.monotonic', return_value=100.), patch('time.time', return_value=100.):
            node.tick()  # No monitor heartbeat or sensor data required.
            node.driver.drive.assert_called_with(0.4, 0.2, 0.1)
            node.on_safety_stop(None, None)
            node.tick()
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.driver.safety_stop.assert_called()
            node.driver.safety_sample = (True, 100.)
            node.on_safety_reset(None, None)
            node.tgt_speed = [0.4, 0., 0.]
            node.tick()  # Do not drive until reset is acknowledged.
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.driver.safety_sample = (False, 100.)
            node.tick()
            node.driver.drive.assert_called_with(0.4, 0., 0.)
            node.driver.safety_sample = (False, 99.)
            node.tick()  # Missing Pi feedback still blocks propulsion.
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.driver.safety_sample = (False, 100.)
            node.lastSpeedReceived = 98.
            node.tick()  # Stale cmd_vel still zeros the command.
            node.driver.drive.assert_called_with(0., 0., 0.)
