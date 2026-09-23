"""Stop an unused lidar without counting the scan filter as an end consumer."""
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


def consumer_names(raw, filtered, filter_node):
    def name(endpoint):
        return '/' + '/'.join((endpoint.node_namespace.strip('/'),
                               endpoint.node_name)).strip('/')
    return {name(e) for e in filtered} | {
        name(e) for e in raw if name(e) != filter_node}


class Demand:
    def __init__(self, now, idle_seconds):
        self.last_demand = now
        self.idle_seconds = idle_seconds

    def wanted(self, now, consumers):
        if consumers:
            self.last_demand = now
        return bool(consumers) or now - self.last_demand < self.idle_seconds


class LidarDemand(Node):
    def __init__(self):
        super().__init__('lidar_demand')
        for key, value in [('raw_topic', '/scan_raw'), ('scan_topic', '/scan'),
                           ('filter_node', '/scan_filter'),
                           ('start_service', '/start_motor'),
                           ('stop_service', '/stop_motor'),
                           ('idle_seconds', 5.0)]:
            self.declare_parameter(key, value)
        self.raw_topic = self.get_parameter('raw_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.filter_node = self.get_parameter('filter_node').value
        idle = float(self.get_parameter('idle_seconds').value)
        if not 0 < idle < float('inf'):
            raise ValueError('idle_seconds must be positive and finite')
        self.policy = Demand(time.monotonic(), idle)
        self.clients = {
            True: self.create_client(Empty, self.get_parameter('start_service').value),
            False: self.create_client(Empty, self.get_parameter('stop_service').value),
        }
        self.publisher_ids = None
        self.acknowledged = None
        self.pending = None
        self.retry_after = 0.0
        self.last_warning = -float('inf')
        self.names = None
        self.create_timer(0.5, self.tick)

    def warn(self, message, now):
        if now - self.last_warning >= 10.0:
            self.get_logger().warning(message)
            self.last_warning = now

    def tick(self):
        now = time.monotonic()
        names = consumer_names(
            self.get_subscriptions_info_by_topic(self.raw_topic),
            self.get_subscriptions_info_by_topic(self.scan_topic), self.filter_node)
        if names != self.names:
            self.get_logger().info('Lidar consumers: ' + (', '.join(sorted(names)) or 'none'))
            self.names = names
        wanted = self.policy.wanted(now, names)

        # A restarted driver starts its motor again. Reapply the policy once its
        # new publisher appears, even if demand has not changed.
        ids = frozenset(bytes(e.endpoint_gid) for e in
                        self.get_publishers_info_by_topic(self.raw_topic))
        if ids != self.publisher_ids:
            self.publisher_ids = ids
            self.acknowledged = None
            if self.pending:
                client, future, _, _ = self.pending
                client.remove_pending_request(future)
                self.pending = None
            self.retry_after = now

        if self.pending:
            client, future, target, sent = self.pending
            if future.done():
                self.pending = None
                try:
                    future.result()
                    self.acknowledged = target
                    self.get_logger().info(
                        'Lidar {} service completed'.format('start' if target else 'stop'))
                except Exception as exc:
                    self.warn('Lidar motor service failed: ' + str(exc), now)
                    self.retry_after = now + 2.0
            elif now - sent >= 10.0:
                client.remove_pending_request(future)
                self.pending = None
                self.acknowledged = None
                self.retry_after = now + 2.0
                self.warn('Lidar motor service timed out; will retry', now)
            else:
                return

        if not ids or wanted == self.acknowledged or now < self.retry_after:
            return
        client = self.clients[wanted]
        if not client.service_is_ready():
            self.warn('Waiting for lidar ' + ('start' if wanted else 'stop') +
                      ' motor service', now)
            return
        self.pending = (client, client.call_async(Empty.Request()), wanted, now)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDemand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
