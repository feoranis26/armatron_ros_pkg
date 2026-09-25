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
        node.motion_blocked = False
        node.inhibit_requested = None
        node.last_safety_send = -float('inf')
        node.last_safety_sample = None
        node.driver = Mock(safety_sample=(False, 100.), position=[0., 0.], speed=[0., 0.])
        node.safety_publisher = Mock()
        node.gyro = NS(angle=0., sample=None)
        node.gyro_seen = False
        node.heading_ready = True
        node.heading_ready_at = 100.
        node.heading_seen = True
        node.gyro_imu = Mock(tracking_valid=True, rejections=0)
        node.gyro_valid_publisher = Mock()
        node.gyro_raw_publisher = Mock()
        node.last_raw_gyro_at = None
        node.last_gyro_rejections = 0
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
            node.tick()  # No motion-consistency heartbeat required.
            node.driver.drive.assert_called_with(0.4, 0.2, 0.1)
            node.driver.drive.return_value = False
            node.tick()  # A denied UDP send drops the old motion command.
            self.assertTrue(node.motion_blocked)
            self.assertEqual(node.tgt_speed, [0., 0., 0.])
            node.driver.drive.return_value = True
            node.tick()  # Recovery sends zero even with healthy telemetry.
            node.driver.drive.assert_called_with(0., 0., 0.)
            self.assertFalse(node.motion_blocked)
            command = Twist()
            command.linear.x = 0.15
            node.on_vel_msg_received(command)
            node.tick()  # Only new command input resumes motion.
            node.driver.drive.assert_called_with(0.15, 0., 0.)
            node.gyro.sample = (120., 100.)
            node.gyro_imu.tracking_valid = False
            node.gyro_imu.rejections = 1
            node.tick()  # Fresh UDP packets with invalid heading still stop immediately.
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.gyro_raw_publisher.publish.assert_called_once()
            self.assertEqual(node.gyro_raw_publisher.publish.call_args.args[0].data, 120.)
            node.get_logger().error.assert_called()
            self.assertIsNone(node.inhibit_requested)  # No persistent latch.
            node.gyro_imu.tracking_valid = True
            node.tick()
            node.gyro_raw_publisher.publish.assert_called_once()  # No duplicate raw packet.
            node.on_safety_stop(None, None)
            node.tick()
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.gyro.angle = None
            node.tick()
            node.gyro.angle = 0.
            node.tick()
            self.assertTrue(node.inhibit_requested)  # Explicit stop survives recovery.
            node.lastSpeedReceived = 100.
            node.inhibit_requested = False
            node.gyro.angle = None
            node.tick()
            self.assertFalse(node.inhibit_requested)
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.on_safety_reset(None, None)
            self.assertFalse(node.inhibit_requested)
            node.gyro.angle = 0.
            node.tgt_speed = [0.4, 0., 0.]
            node.tick()
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.driver.safety_stop.assert_called()
            node.driver.safety_sample = (True, 100.)
            node.on_safety_reset(None, None)
            node.tgt_speed = [0.4, 0., 0.]
            node.tick()  # Do not drive until reset is acknowledged.
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.driver.safety_sample = (False, 100.)
            node.tick()  # Transition clears pre-recovery command.
            node.tgt_speed = [0.4, 0., 0.]
            node.tick()
            node.driver.drive.assert_called_with(0.4, 0., 0.)
            node.driver.safety_sample = (False, 99.)
            node.tick()  # Missing Pi feedback still blocks propulsion.
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.driver.safety_sample = (False, 100.)
            node.lastSpeedReceived = 98.
            node.tick()  # Stale cmd_vel still zeros the command.
            node.driver.drive.assert_called_with(0., 0., 0.)
            node.inhibit_requested = False
            node.heading_ready_at = 99.
            node.lastSpeedReceived = 100.
            node.tick()  # Guard death blocks drive even with fresh gyro packets.
            self.assertFalse(node.inhibit_requested)
            node.driver.drive.assert_called_with(0., 0., 0.)
