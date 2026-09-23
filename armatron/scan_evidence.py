"""Independent planar scan evidence; no ROS, motor commands, or EKF writes.

Information is dimensionless geometry strength, not calibrated covariance.
All candidate transforms map current base-frame points into the reference base.
"""
import math
import numpy as np


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def relative_pose(first, last):
    xy = rotation(-first[2]) @ (np.array(last[:2])-first[:2])
    yaw = math.atan2(math.sin(last[2]-first[2]), math.cos(last[2]-first[2]))
    return np.array([xy[0], xy[1], yaw])


class ScanEvidence:
    def __init__(self, max_points=240, min_points=30, max_gap=0.5,
                 match_distance=0.4, residual_cap=0.25, min_overlap=0.55,
                 max_error=0.12, weak_ratio=0.03, min_strength=0.005,
                 rotation_scale=2.0, score_margin=0.025, directional_ratio=0.20, max_overlap_loss=0.25):
        self.__dict__.update(locals())
        del self.self
        if max_points < min_points or min_points < 6:
            raise ValueError('Require max_points >= min_points >= 6')
        if not all(math.isfinite(v) and v > 0 for v in
                   (max_gap, match_distance, residual_cap, max_error, weak_ratio,
                    min_strength, rotation_scale, score_margin, directional_ratio)):
            raise ValueError('Evidence scales must be finite and positive')
        if not 0 < min_overlap <= 1:
            raise ValueError('min_overlap must be in (0, 1]')
        if not 0 <= max_overlap_loss < 1:
            raise ValueError('max_overlap_loss must be in [0, 1)')
        if directional_ratio > 1:
            raise ValueError('directional_ratio must be in (0, 1]')

    def analyze(self, reference, current, hypotheses):
        reference = np.asarray(reference, dtype=float)
        current = np.asarray(current, dtype=float)
        failure = {'state': 'TRACKING_UNRELIABLE', 'reason': 'insufficient scan geometry'}
        if reference.ndim != 2 or current.ndim != 2 or reference.shape[1] != 2 or current.shape[1] != 2:
            return failure
        # NaN placeholders preserve beam adjacency across invalid ranges.
        if len(reference) < 3:
            return failure
        delta = reference[2:] - reference[:-2]
        length = np.linalg.norm(delta, axis=1)
        finite = np.isfinite(reference[:-2]).all(axis=1) & np.isfinite(reference[1:-1]).all(axis=1) & np.isfinite(reference[2:]).all(axis=1)
        connected = ((np.linalg.norm(reference[1:-1]-reference[:-2], axis=1) < self.max_gap) &
                     (np.linalg.norm(reference[2:]-reference[1:-1], axis=1) < self.max_gap))
        valid = finite & connected & (length > 1e-5)
        target = reference[1:-1][valid]
        tangent = delta[valid] / length[valid, None]
        normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        points = current[np.isfinite(current).all(axis=1)]
        if min(len(points), len(target)) < self.min_points:
            return failure
        # Bound work while preserving geometry across the entire scan.
        ids = np.linspace(0, len(points)-1, min(len(points), self.max_points)).astype(int)
        points = points[ids]
        ids = np.linspace(0, len(target)-1, min(len(target), self.max_points*2)).astype(int)
        target, normals = target[ids], normals[ids]

        if any(k not in hypotheses or np.shape(hypotheses[k]) != (3,) or
               not np.isfinite(hypotheses[k]).all() for k in ('rf2o', 'drive', 'zero')):
            return {'state': 'UNAVAILABLE', 'reason': 'invalid motion hypothesis'}
        poses = {k: np.asarray(hypotheses[k], dtype=float).copy() for k in ('rf2o', 'drive', 'zero')}
        # Test translation with the SAME independent gyro rotation. Otherwise a
        # yaw discrepancy can masquerade as evidence against drive translation.
        rf_yaw = poses['rf2o'][2]
        poses['rf2o'][2] = poses['zero'][2]
        poses['drive'][2] = poses['zero'][2]

        def match(pose):
            moved = points @ rotation(pose[2]).T + pose[:2]
            distances = np.sum((moved[:, None, :]-target[None, :, :])**2, axis=2)
            indices = distances.argmin(axis=1)
            n = normals[indices]
            residual = np.sum(n*(moved-target[indices]), axis=1)
            matched = distances[np.arange(len(points)), indices] < self.match_distance**2
            return moved, n, residual, matched

        matches = {k: match(v) for k, v in poses.items()}
        # Use the intersection once, not a different set of inliers for each
        # hypothesis. Missing overlap is a separate reliability failure, never
        # a constant penalty that flattens the differences between scores.
        support = np.logical_and.reduce([v[3] for v in matches.values()])
        count = int(support.sum())
        overlap = float(support.mean())
        out = {'points': len(points), 'common_points': count, 'common_overlap': overlap}
        if (count < self.min_points or overlap < self.min_overlap or
                max(float(v[3].mean()) for v in matches.values())-overlap > self.max_overlap_loss):
            return dict(out, state='TRACKING_UNRELIABLE', reason='insufficient common scan overlap')
        errors = {k: np.minimum(v[2][support]**2, self.residual_cap**2) for k, v in matches.items()}
        mse = {k: float(v.mean()) for k, v in errors.items()}
        best = min(mse, key=mse.get)
        scores = {k: {'error': math.sqrt(mse[k]), 'overlap': float(matches[k][3].mean())} for k in poses}
        moved, n, residual, _ = matches[best]
        inliers = support & (np.abs(residual) < self.max_error)
        out.update(scores=scores, best=best, inliers=int(inliers.sum()))
        if math.sqrt(mse[best]) > self.max_error or inliers.sum() < self.min_points:
            return dict(out, state='TRACKING_UNRELIABLE', reason='poor common-support alignment')
        p, normals_fit = moved[inliers], n[inliers]
        j = np.column_stack((normals_fit, (normals_fit[:,1]*p[:,0]-normals_fit[:,0]*p[:,1])/self.rotation_scale))
        info = j.T @ j / len(j)
        eigenvalues, eigenvectors = np.linalg.eigh(info)
        translation_values, translation_vectors = np.linalg.eigh(info[:2,:2])
        weak = eigenvalues < max(self.min_strength, eigenvalues[-1]*self.weak_ratio)
        difference = poses['drive'][:2]-poses['rf2o'][:2]
        # Inspect the disagreement direction; if there is no material difference,
        # inspect travel direction, or the weakest translation direction at rest.
        direction = (difference if np.linalg.norm(difference) > self.score_margin else poses['drive'][:2])
        if np.linalg.norm(direction) <= self.score_margin:
            direction = translation_vectors[:,0]
        direction = direction/np.linalg.norm(direction)
        directional_strength = float(direction @ info[:2,:2] @ direction)
        directional_ratio = directional_strength/max(float(translation_values[-1]), 1e-12)
        # Curvature uses fixed correspondences and common support. This measures
        # sensitivity, not a difference of RMS scores with a background offset.
        sensitivity = float(np.mean((normals_fit @ direction)**2))
        out.update(eigenvalues=eigenvalues.tolist(), weak_directions=eigenvectors[:,weak].T.tolist(),
                   translation_eigenvalues=translation_values.tolist(),
                   weak_translation_direction=translation_vectors[:,0].tolist(),
                   tested_direction=direction.tolist(), directional_strength=directional_strength,
                   directional_ratio=directional_ratio, probe_curvature=sensitivity,
                   rotation_scale=self.rotation_scale, information=info.tolist())
        if directional_strength < self.min_strength or directional_ratio < self.directional_ratio:
            return dict(out, state='LIDAR_UNDERCONSTRAINED', reason='weak constraints along tested translation direction')
        if abs(math.atan2(math.sin(rf_yaw-poses['zero'][2]), math.cos(rf_yaw-poses['zero'][2]))) > 0.15:
            return dict(out, state='TRACKING_UNRELIABLE', reason='RF2O and gyro rotation disagree')

        def rejected(key):
            paired = errors[key]-errors[best]
            # A robust paired improvement must exceed both a physical noise
            # floor and sampling uncertainty. Adjacent beams are correlated:
            # cap effective sample count rather than treating every ray as iid.
            uncertainty = 3.*float(paired.std())/math.sqrt(min(20, count))
            return float(paired.mean()) > max(self.score_margin**2, uncertainty)

        if rejected('rf2o'):
            return dict(out, state='TRACKING_UNRELIABLE', reason='scan evidence rejects RF2O translation')
        if rejected('drive'):
            return dict(out, state='MOTION_CONTRADICTED', reason='paired scan evidence rejects drive translation',
                        near_zero_supported=not rejected('zero'),
                        motion_kind=('EXTERNAL_MOTION' if np.linalg.norm(poses['drive'][:2]) < self.score_margin
                                     and np.linalg.norm(poses['rf2o'][:2]) > 2*self.score_margin
                                     else 'DRIVE_MOTION_REJECTED'))
        return dict(out, state='CONSISTENT', reason='drive translation compatible with scan evidence')


class EvidenceDwell:
    """Only distinct, consecutive scan stamps count; stale evidence is external."""
    def __init__(self, seconds=0.5, samples=3):
        self.seconds, self.samples = seconds, samples
        self.state, self.candidate = 'UNAVAILABLE', None
        self.since, self.last, self.count = None, None, 0

    def update(self, stamp, candidate):
        if self.last is not None and stamp <= self.last:
            return self.state
        gap = self.last is not None and stamp-self.last > 1.0
        self.last = stamp
        if gap:
            self.state = 'UNAVAILABLE'
        if gap or candidate != self.candidate:
            self.candidate, self.since, self.count = candidate, stamp, 0
        self.count += 1
        if candidate == 'UNAVAILABLE':
            self.state = candidate
        elif self.count >= self.samples and stamp-self.since >= self.seconds:
            self.state = candidate
        return self.state
