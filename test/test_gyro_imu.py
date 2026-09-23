import math
import unittest
from types import SimpleNamespace
from armatron.gyro_imu import GyroImuAdapter


def imu_message():
    return SimpleNamespace(header=SimpleNamespace(), orientation=SimpleNamespace(),
                           angular_velocity_covariance=[0.]*9,
                           linear_acceleration_covariance=[0.]*9)


class GyroImuTest(unittest.TestCase):
    def test_freshness_and_fields(self):
        adapter = GyroImuAdapter(0.0025, imu_message)
        self.assertIsNone(adapter.message(None, 10, None, 'base_link'))
        self.assertIsNone(adapter.message((90., 8), 10, None, 'base_link'))
        msg = adapter.message((90., 10), 10, 'stamp', 'base_link')
        self.assertAlmostEqual(msg.orientation.z, -math.sqrt(0.5))
        self.assertAlmostEqual(msg.orientation.w, math.sqrt(0.5))
        self.assertEqual(msg.header.frame_id, 'base_link')
        self.assertEqual(msg.orientation_covariance[8], 0.0025)
        self.assertEqual(msg.angular_velocity_covariance[0], -1.)
        self.assertEqual(msg.linear_acceleration_covariance[0], -1.)
        self.assertIsNone(adapter.message((90., 10), 10.1, None, 'base_link'))
        self.assertIsNotNone(adapter.message((90., 10.2), 10.2, None, 'base_link'))

    def test_uncertainty_validation(self):
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                GyroImuAdapter(value, imu_message)
