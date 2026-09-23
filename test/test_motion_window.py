import math
import unittest

from armatron.motion_window import MotionWindow, Recovery


class WindowTest(unittest.TestCase):
    def series(self, speed, measured_speed, yaw=0):
        w = MotionWindow()
        for i in range(61):
            t = i*0.05
            w.add('drive', t, (speed, 0., 0.))
            if i % 2 == 0:
                w.add('rf', t, (measured_speed*t*math.cos(yaw),
                               measured_speed*t*math.sin(yaw), yaw))
        return w

    def test_healthy_fast_reverse_and_rotated_translation(self):
        for speed in (0.04, 0.4, -0.4):
            for yaw in (0, math.pi/2):
                for seconds in (0.75, 2.5):
                    self.assertFalse(self.series(speed, speed, yaw).compare(seconds)['bad'])

    def test_slow_stall_and_carrying(self):
        self.assertTrue(self.series(0.02, 0.).compare(2.5)['bad'])
        self.assertTrue(self.series(0.4, 0.).compare(0.75)['bad'])
        self.assertTrue(self.series(0., 0.1).compare(2.5)['bad'])

    def test_stationary_scene_noise_and_larger_carrying_motion(self):
        w = MotionWindow()
        for i in range(61):
            t = i*0.05
            w.add('drive', t, (0., 0., 0.))
            w.add('rf', t, (0.03*math.sin(3*t), 0., 0.08*math.sin(3*t)))
        for seconds in (0.75, 2.5):
            self.assertFalse(w.compare(seconds)['bad'])
        self.assertTrue(self.series(0., 0.2).compare(0.75)['bad'])
        # The stationary allowance must not mask low-speed propulsion stalls.
        self.assertTrue(self.series(0.02, 0.).compare(2.5)['bad'])

    def test_missing_samples_and_out_of_order(self):
        w = self.series(0.4, 0.4)
        self.assertFalse(w.add('rf', 2., (0.,0.,0.)))
        self.assertFalse(w.add('rf', 4., (float('nan'),0.,0.)))
        w.rf = type(w.rf)(r for r in w.rf if r[0] < 2 or r[0] > 2.7)
        self.assertIsNone(w.compare(2.5))


class RecoveryTest(unittest.TestCase):
    def test_acknowledged_stop_and_automatic_rearm(self):
        r = Recovery()
        self.assertTrue(r.step(0, True, True, False, False))
        self.assertEqual(r.state, 'STOPPING')
        self.assertTrue(r.step(1, True, False, True, None))
        self.assertTrue(r.step(2, True, False, True, True))
        self.assertEqual(r.state, 'STALLED')
        self.assertTrue(r.step(4, True, False, True, True))
        self.assertFalse(r.step(5, True, False, True, True))
        self.assertEqual(r.state, 'RESETTING')
        self.assertFalse(r.step(6, True, False, True, True))
        self.assertIsNone(r.step(7, True, False, True, False))
        self.assertEqual(r.state, 'NORMAL')

    def test_no_rearm_into_command_or_stale_sensor(self):
        r = Recovery()
        r.step(0, True, True, False, False)
        for t in range(1,10):
            self.assertTrue(r.step(t, True, False, False, True))
        r.manual_reset = True
        self.assertTrue(r.step(10, False, False, True, True))
        self.assertNotEqual(r.state, 'NORMAL')

    def test_manual_mode_and_abort_reset(self):
        r = Recovery(auto_rearm=False)
        r.step(0, True, True, False, False)
        r.step(1, True, False, True, True)
        self.assertTrue(r.step(10, True, False, True, True))
        r.manual_reset = True
        self.assertFalse(r.step(11, True, False, True, True))
        self.assertTrue(r.step(12, True, False, False, True))
        self.assertEqual(r.state, 'STOPPING')
