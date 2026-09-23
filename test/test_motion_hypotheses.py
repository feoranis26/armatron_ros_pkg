import math
import unittest
import numpy as np
from armatron.motion_hypotheses import History, hypotheses


class HypothesesTest(unittest.TestCase):
    def test_gyro_rotation_and_strafe_use_same_reference_frame(self):
        drive, gyro, rf = History(), History(), History()
        for i in range(21):
            t = i*0.1
            drive.add(t, (0., 0.2, 0.))
            gyro.add(t, (0., 0., math.pi/2))
            rf.add(t, (3.-0.2*t, 4., math.pi/2))
        result = hypotheses(drive, gyro, rf, 0.5, 1.5)
        np.testing.assert_allclose(result['drive'], [0., 0.2, 0.], atol=1e-10)
        np.testing.assert_allclose(result['rf2o'], [0., 0.2, 0.], atol=1e-10)
        np.testing.assert_allclose(result['zero'], [0., 0., 0.], atol=1e-10)

    def test_missing_gap_out_of_order_and_extrapolation_rejected(self):
        h = History()
        for t in (0., 0.1, 0.2, 1., 1.1):
            h.add(t, (0., 0., 0.))
        h.add(0.3, (0., 0., 0.))
        h.add(1.2, (float('nan'), 0., 0.))
        self.assertEqual(len(h.rows), 5)
        self.assertFalse(h.covers(0., 1.))
        self.assertIsNone(hypotheses(h, h, h, -0.1, 0.1))

    def test_heading_wrap(self):
        h = History()
        h.add(0., (0., 0., math.pi-0.1))
        h.add(0.2, (0., 0., -math.pi+0.1))
        self.assertAlmostEqual(h.at(0.1)[2], math.pi)
