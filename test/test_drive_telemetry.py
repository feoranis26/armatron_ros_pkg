"""Check the command/telemetry boundary against Pi wheel kinematics."""
import unittest

from armatron.drive_protocol import WheelDriver, ROTS_PER_MPS, ROTS_PER_RADS_PS


class DriveTelemetryTest(unittest.TestCase):
    def test_body_velocity_round_trip(self):
        # Simulate settled Pi wheel speeds for ROS body-frame commands.
        for vx, vy, wz in [(0.2, 0., 0.), (-0.2, 0., 0.),
                           (0., 0.1, 0.), (0., 0., 0.6),
                           (0.1, -0.1, -0.4)]:
            x, y, th = vx * ROTS_PER_MPS, -vy * ROTS_PER_MPS, wz * ROTS_PER_RADS_PS
            fl, fr, br, bl = -(x+y+th), x-y-th, x+y-th, -(x-y+th)
            telemetry = ((fl-fr-br+bl)/4, (fl+fr-br-bl)/4, (fl+fr+br+bl)/4)
            driver = WheelDriver.__new__(WheelDriver)
            driver.speed = [0., 0., 0.]
            driver.process(['spd', ','.join(str(v) for v in telemetry)])
            for actual, expected in zip(driver.speed, (vx, vy, wz)):
                self.assertAlmostEqual(actual, expected)


if __name__ == '__main__':
    unittest.main()
