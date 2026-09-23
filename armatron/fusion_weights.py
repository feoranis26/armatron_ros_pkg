"""Bounded, directional velocity uncertainty; no propulsion decisions."""
import math
import numpy as np
from .scan_evidence import rotation


class PoseVelocity:
    def __init__(self):
        self.previous = None

    def update(self, stamp, xy, yaw, frame):
        if not np.isfinite([stamp, *xy, yaw]).all():
            return None
        old = self.previous
        if old is not None and stamp <= old[0]:
            return None
        self.previous = (stamp, np.array(xy), yaw, frame)
        if old is None or frame != old[3] or not 0.02 <= stamp-old[0] <= 0.5:
            return None
        velocity = rotation(-yaw) @ (np.array(xy)-old[1])/(stamp-old[0])
        # A discontinuity is not a velocity measurement.
        return velocity if np.linalg.norm(velocity) <= 2.0 else None


class FusionWeights:
    def __init__(self, lidar_variance=0.0025, weak_variance=1.0,
                 wheel_variance=0.04, weak_ratio=0.20, max_age=0.8):
        if not all(math.isfinite(x) and x > 0 for x in
                   (lidar_variance, weak_variance, wheel_variance, weak_ratio, max_age)):
            raise ValueError('Fusion settings must be finite and positive')
        if not lidar_variance < wheel_variance < weak_variance or weak_ratio > 1:
            raise ValueError('Require lidar variance < wheel variance < weak variance, and ratio <= 1')
        self.lidar_variance, self.weak_variance = lidar_variance, weak_variance
        self.wheel_variance, self.weak_ratio, self.max_age = wheel_variance, weak_ratio, max_age

    def get(self, evidence, yaw, stamp):
        unavailable = (np.eye(2)*self.weak_variance, None, 'NO_CONFIDENCE')
        if not isinstance(evidence, dict):
            return unavailable
        try:
            age = stamp-float(evidence['stamp'])
            if evidence.get('schema') != 2 or evidence.get('frame_id') != 'base_link' or not -0.1 <= age <= self.max_age:
                return unavailable
            state, candidate = evidence['state'], evidence['candidate_state']
            usable = {'CONSISTENT', 'LIDAR_UNDERCONSTRAINED', 'MOTION_CONTRADICTED'}
            if candidate not in usable or evidence.get('fusion_state') not in usable:
                return unavailable
            matrix = np.asarray(evidence['fusion_information'], dtype=float)
            reference_yaw = float(evidence['fusion_reference_yaw'])
            if matrix.shape != (3,3) or not np.isfinite(matrix).all() or not math.isfinite(reference_yaw+yaw):
                return unavailable
            matrix = matrix[:2,:2]
            if not np.allclose(matrix, matrix.T, atol=1e-6):
                return unavailable
            values, vectors = np.linalg.eigh(matrix)
            if values[0] < -1e-8 or values[-1] <= 0:
                return unavailable
            weak = (values < 0.005) | (values < values[-1]*self.weak_ratio)
            basis = rotation(reference_yaw-yaw) @ vectors
            rf = basis @ np.diag(np.where(weak, self.weak_variance, self.lidar_variance)) @ basis.T
            # A candidate contradiction vetoes fallback immediately. Enter only
            # after confirmed underconstraint, never merely a missing sensor.
            fallback = state == candidate == 'LIDAR_UNDERCONSTRAINED' and weak.any()
            wheel = (basis @ np.diag(np.where(weak, self.wheel_variance, 1e6)) @ basis.T
                     if fallback else None)
            return rf, wheel, 'WHEEL_FALLBACK' if fallback else 'LIDAR_WEAK' if weak.any() else 'LIDAR'
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError):
            return unavailable


def twist_covariance(xy):
    covariance = np.eye(6)*1e6
    covariance[:2,:2] = xy
    return covariance.ravel().tolist()
