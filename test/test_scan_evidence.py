import math
import unittest
import numpy as np

from armatron.scan_evidence import ScanEvidence, EvidenceDwell, rotation, relative_pose


def room_scan(origin=(0., 0.), corridor=False):
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    distances = np.full(720, np.inf)
    walls = [(1, -2.), (1, 2.)] if corridor else [(0, -3.), (0, 4.), (1, -2.), (1, 2.)]
    for axis, boundary in walls:
        with np.errstate(divide='ignore', invalid='ignore'):
            d = (boundary-origin[axis])/rays[:, axis]
        distances = np.minimum(distances, np.where(d > 0, d, np.inf))
    distances[distances > 12.] = np.nan
    return rays*distances[:, None]


class EvidenceTest(unittest.TestCase):
    def setUp(self):
        self.engine = ScanEvidence()

    def analyze(self, current, rf, drive, corridor=False):
        return self.engine.analyze(room_scan(corridor=corridor), current,
                                   {'rf2o': rf, 'drive': drive, 'zero': [0., 0., 0.]})

    def test_room_consistency_and_stall_evidence(self):
        result = self.analyze(room_scan((0.2, 0.)), [0.2, 0., 0.], [0.2, 0., 0.])
        self.assertEqual(result['state'], 'CONSISTENT', result)
        result = self.analyze(room_scan(), [0., 0., 0.], [0.3, 0., 0.])
        self.assertEqual(result['state'], 'MOTION_CONTRADICTED', result)
        self.assertTrue(result['near_zero_supported'])

    def test_corridor_is_unknown_not_stalled(self):
        result = self.analyze(room_scan((0.3, 0.), True), [0., 0., 0.], [0.3, 0., 0.], True)
        self.assertEqual(result['state'], 'LIDAR_UNDERCONSTRAINED', result)
        self.assertGreater(abs(result['weak_translation_direction'][0]), 0.99)

    def test_underestimated_rf2o_in_rich_room_is_tracking_error(self):
        result = self.analyze(room_scan((0.3, 0.)), [0., 0., 0.], [0.3, 0., 0.])
        self.assertEqual(result['state'], 'TRACKING_UNRELIABLE', result)

    def test_missing_overlap_and_invalid_ranges(self):
        result = self.analyze(room_scan()+20., [0., 0., 0.], [0.3, 0., 0.])
        self.assertEqual(result['state'], 'TRACKING_UNRELIABLE')
        result = self.analyze(np.full((720, 2), np.nan), [0., 0., 0.], [0., 0., 0.])
        self.assertEqual(result['state'], 'TRACKING_UNRELIABLE')

    def test_rotated_base_hypothesis_and_gyro_rotation(self):
        pose = relative_pose([10., 20., math.pi/2], [10., 20.2, math.pi/2])
        np.testing.assert_allclose(pose, [0.2, 0., 0.], atol=1e-10)
        current = room_scan() @ rotation(-0.2).T
        result = self.engine.analyze(room_scan(), current,
                 {k: [0., 0., 0.2] for k in ('rf2o', 'drive', 'zero')})
        self.assertEqual(result['state'], 'CONSISTENT', result)

    def test_distinct_samples_and_persistence(self):
        dwell = EvidenceDwell()
        for t in (0., 0.1, 0.2):
            self.assertEqual(dwell.update(t, 'MOTION_CONTRADICTED'), 'UNAVAILABLE')
        for _ in range(10):
            self.assertEqual(dwell.update(0.2, 'MOTION_CONTRADICTED'), 'UNAVAILABLE')
        self.assertEqual(dwell.update(0.6, 'MOTION_CONTRADICTED'), 'MOTION_CONTRADICTED')
        self.assertEqual(dwell.update(0.7, 'UNAVAILABLE'), 'UNAVAILABLE')

    def test_corridor_sideways_motion_is_observable(self):
        result = self.analyze(room_scan((0., 0.2), True), [0., 0.2, 0.], [0., 0.2, 0.], True)
        self.assertEqual(result['state'], 'CONSISTENT', result)
        self.assertGreater(result['directional_ratio'], 0.9)

    def test_background_unmatched_returns_do_not_imply_flat_geometry(self):
        reference = room_scan()
        current = reference.copy()
        current[::5] *= 3.  # Common missing overlap, not translation uncertainty.
        result = self.engine.analyze(reference, current,
                   {k: [0., 0., 0.] for k in ('rf2o', 'drive', 'zero')})
        self.assertEqual(result['state'], 'CONSISTENT', result)
        self.assertLess(result['common_overlap'], 0.9)

    def test_yaw_disagreement_cannot_be_called_translation_stall(self):
        result = self.analyze(room_scan(), [0., 0., 0.25], [0.3, 0., 0.])
        self.assertEqual(result['state'], 'TRACKING_UNRELIABLE', result)

    def test_candidate_cannot_win_by_dropping_overlap(self):
        result = self.analyze(room_scan(), [0., 0., 0.], [2., 0., 0.])
        self.assertEqual(result['state'], 'TRACKING_UNRELIABLE', result)
