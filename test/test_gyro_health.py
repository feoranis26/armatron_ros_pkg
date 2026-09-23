import unittest
from unittest.mock import Mock, patch
from armatron.gyro_health import GyroHealth
from armatron.gyro_protocol import UDPGyro


class GyroHealthTest(unittest.TestCase):
    def test_missing_stale_and_recovered_packets(self):
        gyro = UDPGyro.__new__(UDPGyro)
        gyro.sample = None
        with patch('time.monotonic', return_value=10):
            self.assertIsNone(gyro.angle)
            gyro.process(['angle', '12.5'])
            self.assertEqual(gyro.angle, 12.5)
            for bad in (['angle'], ['angle', 'nan'], ['angle', 'oops']):
                gyro.process(bad)
            self.assertEqual(gyro.sample, (12.5, 10))
        with patch('time.monotonic', return_value=12):
            self.assertIsNone(gyro.angle)
            gyro.process(['angle', '0'])
            self.assertEqual(gyro.angle, 0.0)

    def test_worker_death_and_sensor_stall(self):
        with patch('time.monotonic', return_value=10):
            health = GyroHealth()
        worker = Mock()
        worker.name = 'sensor'
        worker.is_alive.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'worker exited'):
            health.check([worker])
        worker.is_alive.return_value = True
        with patch('time.monotonic', return_value=16):
            with self.assertRaisesRegex(RuntimeError, 'No valid BNO055'):
                health.check([worker])
            health.last_valid = 16
            health.check([worker])
