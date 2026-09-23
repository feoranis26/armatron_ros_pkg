import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from concurrent.futures import Future


# Exercise the real timer callback without requiring a ROS installation.
spec = importlib.util.spec_from_file_location(
    'lidar_demand_tested', Path(__file__).parents[1] / 'armatron/lidar_demand.py')
module = importlib.util.module_from_spec(spec)
with patch.dict('sys.modules', {
        'rclpy': Mock(), 'rclpy.node': SimpleNamespace(Node=object),
        'std_srvs': Mock(), 'std_srvs.srv': SimpleNamespace(Empty=SimpleNamespace(Request=lambda: None))}):
    spec.loader.exec_module(module)


class LidarDemandTest(unittest.TestCase):
    def test_internal_filter_does_not_keep_motor_on(self):
        endpoint = lambda name: SimpleNamespace(node_namespace='/', node_name=name)
        self.assertEqual(module.consumer_names([endpoint('scan_filter')], [], '/scan_filter'), set())
        self.assertEqual(module.consumer_names([endpoint('scan_filter'), endpoint('recorder')],
                                              [endpoint('rf2o')], '/scan_filter'),
                         {'/recorder', '/rf2o'})

    def test_idle_delay_and_immediate_wakeup(self):
        policy = module.Demand(0., 5.)
        self.assertTrue(policy.wanted(4., []))
        self.assertFalse(policy.wanted(5., []))
        self.assertTrue(policy.wanted(6., ['rviz']))
        self.assertTrue(policy.wanted(10., []))
        self.assertFalse(policy.wanted(11., []))

    def make_node(self):
        node = module.LidarDemand.__new__(module.LidarDemand)
        node.raw_topic, node.scan_topic, node.filter_node = '/scan_raw', '/scan', '/scan_filter'
        node.policy = module.Demand(0., 5.)
        node.publisher_ids, node.acknowledged, node.pending, node.names = None, None, None, None
        node.retry_after, node.last_warning = 0., -float('inf')
        node.get_logger = Mock(return_value=Mock())
        node.get_subscriptions_info_by_topic = Mock(return_value=[])
        node.get_publishers_info_by_topic = Mock(return_value=[SimpleNamespace(endpoint_gid=[1])])
        node.clients = {True: Mock(), False: Mock()}
        for client in node.clients.values():
            client.service_is_ready.return_value = True
            client.call_async.side_effect = lambda _: Future()
        return node

    def test_stop_wakeup_and_driver_restart(self):
        node = self.make_node()
        with patch.object(module.time, 'monotonic', return_value=6.):
            node.tick()
            node.clients[False].call_async.assert_called_once()
            node.pending[1].set_result(None)
            node.tick()
            self.assertFalse(node.acknowledged)
            node.tick()
            node.clients[False].call_async.assert_called_once()
            node.get_subscriptions_info_by_topic.return_value = [SimpleNamespace(node_namespace='/', node_name='rf2o')]
            node.tick()
            node.clients[True].call_async.assert_called_once()
            node.pending[1].set_result(None)
            node.tick()
            self.assertTrue(node.acknowledged)
            node.get_publishers_info_by_topic.return_value = [SimpleNamespace(endpoint_gid=[2])]
            node.tick()
            self.assertEqual(node.clients[True].call_async.call_count, 2)

    def test_missing_service_and_timeout_retry(self):
        node = self.make_node()
        node.clients[False].service_is_ready.return_value = False
        with patch.object(module.time, 'monotonic', return_value=6.):
            node.tick()
            node.clients[False].call_async.assert_not_called()
            node.clients[False].service_is_ready.return_value = True
            node.tick()
        with patch.object(module.time, 'monotonic', return_value=17.):
            node.tick()
            self.assertIsNone(node.pending)
            self.assertIsNone(node.acknowledged)
        with patch.object(module.time, 'monotonic', return_value=19.):
            node.tick()
            self.assertEqual(node.clients[False].call_async.call_count, 2)
