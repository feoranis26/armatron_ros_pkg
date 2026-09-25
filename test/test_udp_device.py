"""Network startup failures must not permanently disable the UDP bridge."""
import errno
import socket
import unittest
from unittest.mock import Mock, patch

from armatron.udp_device import UDPDevice


class UDPRecoveryTest(unittest.TestCase):
    def device(self):
        with patch('armatron.udp_device.socket.socket') as factory:
            device = UDPDevice('10.8.3.56', 11753, 11754)
        device.port = factory.return_value
        device.logger = Mock()
        return device

    def test_failed_send_is_dropped_and_next_command_recovers(self):
        device = self.device()
        device.port.sendto.side_effect = [PermissionError(errno.EPERM, 'denied'), 4]
        self.assertFalse(device.send('old command'))
        self.assertTrue(device.send('new command'))
        self.assertEqual([c.args[0] for c in device.port.sendto.call_args_list],
                         [b'old command', b'new command'])
        device.logger.warning.assert_called_once()

    def test_error_logging_is_throttled_and_unexpected_errors_raise(self):
        device = self.device()
        device.port.sendto.side_effect = OSError(errno.ENETUNREACH, 'no route')
        with patch('time.monotonic', side_effect=[10., 11., 16.]):
            for _ in range(3):
                self.assertFalse(device.send('connect'))
        self.assertEqual(device.logger.warning.call_count, 2)
        device.port.sendto.side_effect = OSError(errno.EBADF, 'bad descriptor')
        with self.assertRaises(OSError):
            device.send('connect')

    def test_connect_and_receive_recover_without_restarting(self):
        device = self.device()
        device.stop_event = Mock()
        device.port.connect.side_effect = [OSError(errno.ENETUNREACH, 'no route'), None]
        device.process = Mock(side_effect=lambda _: setattr(device, 'stop_flag', True))
        device.port.recvfrom.side_effect = [
            socket.timeout(), PermissionError(errno.EPERM, 'denied'),
            (b'angle:42', ('10.8.3.56', 11753))]
        device.receive_thread()
        self.assertEqual(device.port.connect.call_count, 2)
        device.process.assert_called_once_with(['angle', '42'])
        self.assertEqual(device.stop_event.wait.call_count, 2)

    def test_bind_failure_prevents_threads_starting(self):
        device = self.device()
        device.port.bind.side_effect = OSError(errno.EADDRINUSE, 'in use')
        with patch('armatron.udp_device.threading.Thread') as thread:
            with self.assertRaises(OSError):
                device.start()
            thread.assert_not_called()

    def test_bind_precedes_thread_start(self):
        device = self.device()
        with patch('armatron.udp_device.threading.Thread') as thread:
            thread.side_effect = lambda **kwargs: (
                device.port.bind.assert_called_once_with(('', 11754)) or Mock())
            device.start()
        device.stop()


if __name__ == '__main__':
    unittest.main()
