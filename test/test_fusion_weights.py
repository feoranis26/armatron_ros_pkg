import math
import unittest
import numpy as np
from armatron.fusion_weights import FusionWeights, PoseVelocity, twist_covariance
from armatron.evidence_windows import combine


def evidence(state='LIDAR_UNDERCONSTRAINED', yaw=0.):
    return dict(schema=2, stamp=10., frame_id='base_link', state=state,
                candidate_state=state, fusion_state=state,
                fusion_information=np.diag([0.02, 0.98, 1.]).tolist(), fusion_reference_yaw=yaw)


class FusionTest(unittest.TestCase):
    def test_weak_axis_prefers_wheel_strong_axis_prefers_lidar(self):
        rf, wheel, mode = FusionWeights().get(evidence(), 0., 10.1)
        self.assertEqual(mode, 'WHEEL_FALLBACK')
        self.assertGreater(rf[0,0], wheel[0,0]*10)
        self.assertGreater(wheel[1,1], rf[1,1]*1000)
        # Scalar information-weighted response to lidar=0, steps=0.2.
        self.assertGreater((0.2/wheel[0,0])/(1/rf[0,0]+1/wheel[0,0]), 0.19)

    def test_covariance_rotation_off_diagonals_and_psd(self):
        rf, wheel, _ = FusionWeights().get(evidence(yaw=math.pi/4), 0., 10.)
        for cov in (rf, wheel):
            self.assertTrue(np.all(np.linalg.eigvalsh(cov) > 0))
            self.assertGreater(abs(cov[0,1]), 0.1)
            np.testing.assert_allclose(cov, cov.T)
        c = twist_covariance(rf)
        self.assertEqual(c[1], c[6])
        self.assertEqual(c[35], 1e6)

    def test_stale_invalid_and_candidate_contradiction_disable_fallback(self):
        w = FusionWeights()
        for e in (None, {}, evidence(), dict(evidence(), fusion_information=[[float('nan')]*3]*3)):
            self.assertIsNone(w.get(e, 0., 12.)[1])
        e = evidence()
        e['candidate_state'] = 'MOTION_CONTRADICTED'
        self.assertIsNone(w.get(e, 0., 10.)[1])
        e['fusion_information'] = np.eye(3).tolist()
        self.assertAlmostEqual(w.get(e, 0., 10.)[0][0,0], 0.0025)

    def test_pose_velocity_body_frame_gap_duplicate_and_reset(self):
        p = PoseVelocity()
        self.assertIsNone(p.update(0., [0.,0.], math.pi/2, 'odom'))
        np.testing.assert_allclose(p.update(.1, [0.,.02], math.pi/2, 'odom'), [.2,0.], atol=1e-10)
        self.assertIsNone(p.update(.1, [0.,.02], math.pi/2, 'odom'))
        self.assertIsNone(p.update(1., [0.,.02], math.pi/2, 'odom'))
        self.assertIsNone(p.update(1.1, [20.,0.], 0., 'odom'))

    def test_accumulation_does_not_override_current_weak_geometry(self):
        long = {'state': 'MOTION_CONTRADICTED', 'motion_kind': 'EXTERNAL_MOTION'}
        result = combine({'state': 'CONSISTENT'}, long)
        self.assertEqual(result['state'], 'MOTION_CONTRADICTED')
        self.assertEqual(result['motion_kind'], 'EXTERNAL_MOTION')
        self.assertEqual(combine({'state': 'LIDAR_UNDERCONSTRAINED'}, long)['state'], 'LIDAR_UNDERCONSTRAINED')

    def test_ekf_consumes_velocity_covariance_and_gyro_heading(self):
        from pathlib import Path
        import yaml
        config = yaml.safe_load((Path(__file__).parents[1]/'config/odometry/ekf.yaml').read_text())['ekf_filter_node']['ros__parameters']
        for source in ('odom0', 'odom1'):
            self.assertFalse(config[source+'_differential'])
            self.assertEqual([i for i,v in enumerate(config[source+'_config']) if v], [6,7])
        self.assertEqual([i for i,v in enumerate(config['imu0_config']) if v], [5])
