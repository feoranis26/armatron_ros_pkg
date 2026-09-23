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
        'sensor_msgs.msg': {'Imu': Message, 'LaserScan': Message},
        'std_msgs.msg': {'Bool': Message, 'String': Message},
        'robot_localization.srv': {'ToggleFilterProcessing': NS(Request=Message)},
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
    def test_loss_latches_and_recovery_does_not_clear(self):
        guard = load('heading_guard').HeadingWatchdog(0.)
        self.assertFalse(guard.ready(0.))
        guard.sample(10., 10., .1)
        self.assertTrue(guard.ready(.2))
        guard.sample(10., 10.5, .9)  # Duplicate cannot refresh receipt time.
        self.assertFalse(guard.ready(1.2))
        guard.sample(12., 12., 1.3)
        self.assertFalse(guard.ready(1.3))

    def test_missing_startup_and_old_future_packets(self):
        guard = load('heading_guard').HeadingWatchdog(0.)
        guard.sample(5., 10., 1.)
        guard.sample(12., 10., 2.)
        self.assertIsNone(guard.received)
        self.assertFalse(guard.ready(5.1))
        self.assertTrue(guard.latched)

    def test_latch_persists_stops_drive_and_requests_ekf_off(self):
        module = load('heading_guard')
        node = module.HeadingGuard.__new__(module.HeadingGuard)
        with tempfile.TemporaryDirectory() as directory, patch('time.monotonic', return_value=10.):
            node.marker = Path(directory) / 'heading_fault.json'
            node.watchdog = module.HeadingWatchdog(0.)
            node.recorded = False
            node.pending = None
            node.last_log = node.last_request = -float('inf')
            node.ready_pub = Mock()
            node.stop_pub = Mock()
            node.status_pub = Mock()
            node.toggle = Mock()
            node.get_logger = Mock()
            node.tick()
            self.assertTrue(node.marker.exists())
            self.assertFalse(node.ready_pub.publish.call_args.args[0].data)
            self.assertTrue(node.stop_pub.publish.call_args.args[0].data)
            self.assertFalse(node.toggle.call_async.call_args.args[0].on)
            node.watchdog.sample(11., 11., 11.)
            self.assertFalse(node.watchdog.ready(11.))

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

    def test_fault_blocks_map_save_before_calling_ros(self):
        from armatron.map_manager import command_save
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'heading_fault.json').write_text('{}')
            with patch('armatron.map_manager.subprocess.run') as run:
                with self.assertRaisesRegex(RuntimeError, 'heading fault'):
                    command_save(NS(root=root, timeout=20.))
                run.assert_not_called()
