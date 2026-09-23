import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock, patch


def load(name):
    modules = {}
    class Message:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)
    for key, attrs in {
        'rclpy': {}, 'rclpy.node': {'Node': object},
        'rclpy.time': {'Time': object},
        'tf2_ros': {'Buffer': object, 'TransformListener': object, 'TransformException': Exception},
        'nav_msgs.msg': {'Odometry': Message},
        'sensor_msgs.msg': {'Imu': Message, 'LaserScan': Message},
        'std_msgs.msg': {'Bool': Message, 'String': Message},
        'robot_localization.srv': {'ToggleFilterProcessing': NS(Request=Message), 'SetPose': NS(Request=lambda: NS(pose=NS()))},
    }.items():
        modules[key] = ModuleType(key)
        modules[key].__dict__.update(attrs)
    spec = importlib.util.spec_from_file_location('armatron._'+name+'_test',
        Path(__file__).parents[1] / 'armatron' / (name+'.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class HeadingGuardTest(unittest.TestCase):
    def test_loss_and_automatic_recovery(self):
        guard = load('heading_guard').HeadingWatchdog(0.)
        self.assertFalse(guard.ready(0.))
        guard.sample(10., 10., .1)
        self.assertFalse(guard.ready(.2))
        guard.sample(10.3, 10.3, .4)
        self.assertTrue(guard.ready(.5))
        guard.sample(10.3, 10.5, 1.)  # Duplicate cannot refresh receipt time.
        self.assertFalse(guard.ready(1.5))
        guard.sample(12., 12., 1.6)
        self.assertFalse(guard.ready(1.6))
        guard.sample(12.4, 12.4, 2.)
        self.assertTrue(guard.ready(2.))

    def test_missing_startup_and_old_future_packets(self):
        guard = load('heading_guard').HeadingWatchdog(0.)
        guard.sample(5., 10., 1.)
        guard.sample(12., 10., 2.)
        self.assertIsNone(guard.received)
        self.assertFalse(guard.ready(50.))
        guard.sample(51., 51., 51.)
        guard.sample(51.4, 51.4, 51.4)
        self.assertTrue(guard.ready(51.4))

    def test_pause_reset_preserved_pose_then_resume(self):
        module = load('heading_guard')
        node = module.HeadingGuard.__new__(module.HeadingGuard)
        node.watchdog = module.HeadingWatchdog(0.)
        node.phase = 'RUNNING'
        node.pending = None
        node.last_status = None
        node.ready_pub = Mock()
        node.status_pub = Mock()
        node.toggle = Mock()
        node.set_pose = Mock()
        node.get_logger = Mock()
        node.get_clock = Mock()
        node.odom = NS(header=NS(frame_id='odom', stamp=None), pose=NS(x=7., yaw=1.2))
        node.odom_at = 10.
        with patch('time.monotonic', return_value=10.):
            node.tick()
            self.assertFalse(node.toggle.call_async.call_args.args[0].on)
            self.assertFalse(node.ready_pub.publish.call_args.args[0].data)
            node.tick()  # Pause acknowledged, wait for gyro.
            self.assertEqual(node.phase, 'PAUSED')
            node.watchdog.sample(10., 10., 9.5)
            node.watchdog.sample(10.4, 10.4, 9.9)
            node.tick()
            node.tick()  # Submit SetPose before enabling filter.
            request = node.set_pose.call_async.call_args.args[0]
            self.assertEqual(request.pose.pose.x, 7.)
            self.assertEqual(request.pose.pose.yaw, 1.2)
            self.assertFalse(node.ready_pub.publish.call_args.args[0].data)
            node.tick()  # SetPose acknowledged; request toggle on.
            self.assertTrue(node.toggle.call_async.call_args.args[0].on)
            node.tick()
            self.assertTrue(node.ready_pub.publish.call_args.args[0].data)

    def test_lost_gyro_while_enable_is_pending_returns_to_pause(self):
        module = load('heading_guard')
        node = module.HeadingGuard.__new__(module.HeadingGuard)
        node.watchdog = module.HeadingWatchdog(0.)
        node.phase = 'START'
        node.pending = Mock()
        node.pending.done.return_value = True
        node.toggle = Mock()
        node.ready_pub = Mock()
        node.status_pub = Mock()
        node.get_logger = Mock()
        node.last_status = None
        with patch('time.monotonic', return_value=10.):
            node.tick()
        self.assertFalse(node.ready_pub.publish.call_args.args[0].data)
        self.assertFalse(node.toggle.call_async.call_args.args[0].on)

    def test_timed_out_service_retries_pause_without_unlocking(self):
        module = load('heading_guard')
        node = module.HeadingGuard.__new__(module.HeadingGuard)
        node.watchdog = module.HeadingWatchdog(0.)
        node.phase = 'START'
        node.pending = Mock()
        node.pending.done.return_value = False
        client = node.pending_client = Mock()
        node.request_at = 0.
        node.toggle = Mock()
        node.ready_pub = Mock()
        node.status_pub = Mock()
        node.get_logger = Mock()
        node.last_status = None
        with patch('time.monotonic', return_value=10.):
            node.tick()
        client.remove_pending_request.assert_called_once()
        self.assertFalse(node.ready_pub.publish.call_args.args[0].data)
        self.assertFalse(node.toggle.call_async.call_args.args[0].on)

    def test_scan_gate_fails_closed_on_false_or_missing_heartbeat(self):
        module = load('scan_filter')
        node = module.ScanTrimmer.__new__(module.ScanTrimmer)
        node.heading_ready = False
        node.heading_at = 10.
        node.scan_publisher = Mock()
        with patch('time.monotonic', return_value=10.):
            node.on_scan_received(None)  # Must return before accessing scan fields.
        node.heading_ready = True
        with patch('time.monotonic', return_value=10.4):
            node.on_scan_received(None)
        node.scan_publisher.publish.assert_not_called()
