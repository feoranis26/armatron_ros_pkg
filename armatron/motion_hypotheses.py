"""Timestamp-aligned wheel/gyro and lidar motion hypotheses, independent of ROS."""
from collections import deque
import math
import numpy as np
from .motion_window import interpolate
from .scan_evidence import rotation, relative_pose


class History:
    def __init__(self):
        self.rows = deque()

    def add(self, t, values):
        if not all(math.isfinite(x) for x in (t, *values)):
            return
        if self.rows and t <= self.rows[-1][0]:
            return
        self.rows.append((t, *values))
        while self.rows and self.rows[0][0] < t-4.0:
            self.rows.popleft()

    def covers(self, a, b):
        if not self.rows or self.rows[0][0] > a or self.rows[-1][0] < b:
            return False
        return not any(y[0]-x[0] > 0.3 for x, y in zip(self.rows, list(self.rows)[1:])
                       if x[0] < b and y[0] > a)

    def at(self, t):
        return interpolate(self.rows, t, heading=True)


def hypotheses(drive, gyro, rf, a, b):
    if not all(h.covers(a, b) for h in (drive, gyro, rf)):
        return None
    initial_yaw = gyro.at(a)[2]
    dyaw = math.atan2(math.sin(gyro.at(b)[2]-initial_yaw), math.cos(gyro.at(b)[2]-initial_yaw))
    times = sorted({a, b} | {r[0] for r in drive.rows if a < r[0] < b} |
                   {r[0] for r in gyro.rows if a < r[0] < b})
    xy = np.zeros(2)
    for t0, t1 in zip(times, times[1:]):
        v0 = rotation(gyro.at(t0)[2]-initial_yaw) @ drive.at(t0)[:2]
        v1 = rotation(gyro.at(t1)[2]-initial_yaw) @ drive.at(t1)[:2]
        xy += (v0+v1)*0.5*(t1-t0)
    return {'rf2o': relative_pose(rf.at(a), rf.at(b)),
            'drive': np.array([*xy, dyaw]), 'zero': np.array([0., 0., dyaw])}
