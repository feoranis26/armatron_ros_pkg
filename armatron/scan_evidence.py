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
                 rotation_scale=2.0, score_margin=0.025, probe_distance=0.10):
        self.__dict__.update(locals())
        del self.self
        if max_points < min_points or min_points < 6:
            raise ValueError('Require max_points >= min_points >= 6')
        if not all(math.isfinite(v) and v > 0 for v in
                   (max_gap, match_distance, residual_cap, max_error, weak_ratio,
                    min_strength, rotation_scale, score_margin, probe_distance)):
            raise ValueError('Evidence scales must be finite and positive')
        if not 0 < min_overlap <= 1:
            raise ValueError('min_overlap must be in (0, 1]')

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

        def score(pose, details=False):
            moved = points @ rotation(pose[2]).T + pose[:2]
            distances = np.sum((moved[:, None, :]-target[None, :, :])**2, axis=2)
            indices = distances.argmin(axis=1)
            offsets = moved-target[indices]
            n = normals[indices]
            residual = np.sum(n*offsets, axis=1)
            matched = distances[np.arange(len(points)), indices] < self.match_distance**2
            # Every candidate pays for the same points. No free score improvement
            # by dropping mismatches, occluded points, or poor overlap.
            errors = np.where(matched, np.minimum(residual**2, self.residual_cap**2), self.residual_cap**2)
            result = {'error': float(np.sqrt(errors.mean())), 'overlap': float(matched.mean())}
            if details:
                inliers = matched & (np.abs(residual) < self.max_error)
                p, n = moved[inliers], n[inliers]
                jacobian = np.column_stack((n, (n[:, 1]*p[:, 0]-n[:, 0]*p[:, 1])/self.rotation_scale))
                info = jacobian.T @ jacobian / max(1, len(jacobian))
                result.update(inliers=int(inliers.sum()), information=info)
            return result

        if any(np.shape(hypotheses[k]) != (3,) or not np.isfinite(hypotheses[k]).all()
               for k in ('rf2o', 'drive', 'zero')):
            return {'state': 'UNAVAILABLE', 'reason': 'invalid motion hypothesis'}
        scores = {name: score(np.asarray(pose)) for name, pose in hypotheses.items()}
        best = min(scores, key=lambda k: scores[k]['error'])
        fit = score(np.asarray(hypotheses[best]), True)
        out = {'scores': scores, 'best': best, 'points': len(points), 'inliers': fit['inliers']}
        if fit['overlap'] < self.min_overlap or fit['error'] > self.max_error or fit['inliers'] < self.min_points:
            return dict(out, state='TRACKING_UNRELIABLE', reason='poor fit or overlap')
        eigenvalues, eigenvectors = np.linalg.eigh(fit['information'])
        strength = max(self.min_strength, float(eigenvalues[-1])*self.weak_ratio)
        weak = eigenvalues < strength
        # Probe the weakest direction with yaw held by the independent gyro.
        translation_values, translation_vectors = np.linalg.eigh(fit['information'][:2, :2])
        direction = translation_vectors[:, 0]
        probe = np.array([*(direction*self.probe_distance), 0.])
        probe_scores = [score(np.asarray(hypotheses[best])+sign*probe)['error'] for sign in (-1, 1)]
        flat = max(probe_scores)-fit['error'] < self.score_margin
        out.update(eigenvalues=eigenvalues.tolist(), weak_directions=eigenvectors[:, weak].T.tolist(),
                   translation_eigenvalues=translation_values.tolist(),
                   weak_translation_direction=direction.tolist(), probe_errors=probe_scores,
                   rotation_scale=self.rotation_scale, information=fit['information'].tolist())
        if weak.any() or flat:
            return dict(out, state='LIDAR_UNDERCONSTRAINED', reason='weak geometry or ambiguous translation score')
        if scores['rf2o']['error'] > fit['error'] + self.score_margin:
            return dict(out, state='TRACKING_UNRELIABLE', reason='RF2O hypothesis fits worse than an alternative')
        if scores['drive']['error'] > fit['error'] + self.score_margin:
            return dict(out, state='MOTION_CONTRADICTED', reason='scan alignment rejects drive prediction',
                        near_zero_supported=scores['zero']['error'] <= fit['error']+self.score_margin)
        return dict(out, state='CONSISTENT', reason='drive prediction compatible with constrained scans')


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
