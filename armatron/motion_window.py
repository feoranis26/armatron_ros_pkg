"""Time-aligned planar displacement comparison, independent of ROS."""
import bisect
import math
from collections import deque


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def interpolate(rows, t, heading=False):
    times = [row[0] for row in rows]
    i = bisect.bisect_right(times, t)
    if i == 0:
        return rows[0][1:]
    if i == len(rows):
        return rows[-1][1:]
    a, b = rows[i-1], rows[i]
    fraction = (t-a[0])/(b[0]-a[0])
    delta = [y-x for x, y in zip(a[1:], b[1:])]
    if heading:
        delta[2] = angle(delta[2])
    return tuple(x+fraction*d for x, d in zip(a[1:], delta))


class MotionWindow:
    def __init__(self, distance_floor=0.015, relative_error=0.4, angle_floor=0.10,
                 timing_tolerance=0.2, rotation_translation_tolerance=0.125,
                 stationary_distance=0.08, stationary_angle=0.25):
        self.drive = deque()
        self.rf = deque()
        self.distance_floor = distance_floor
        self.relative_error = relative_error
        self.angle_floor = angle_floor
        self.timing_tolerance = timing_tolerance
        self.rotation_translation_tolerance = rotation_translation_tolerance
        self.stationary_distance = stationary_distance
        self.stationary_angle = stationary_angle
        if not all(math.isfinite(v) and v > 0 for v in
                   (stationary_distance, stationary_angle)):
            raise ValueError('Stationary tolerances must be positive and finite')

    def add(self, kind, t, values):
        rows = getattr(self, kind)
        if not all(math.isfinite(v) for v in (t, *values)):
            return False
        if rows and t <= rows[-1][0]:
            return False
        rows.append((t, *values))
        while rows and rows[0][0] < t-6.0:
            rows.popleft()
        return True

    def compare(self, seconds):
        if len(self.rf) < 3 or len(self.drive) < 3:
            return None
        end = min(self.rf[-1][0], self.drive[-1][0])
        start = end-seconds
        if self.rf[0][0] > start or self.drive[0][0] > start:
            return None
        # Missing samples cannot count as evidence of a stationary robot.
        for rows in (self.rf, self.drive):
            relevant = [r[0] for r in rows if start-0.35 <= r[0] <= end+0.35]
            if any(b-a > 0.35 for a, b in zip(relevant, relevant[1:])):
                return None
        times = sorted({start, end} | {r[0] for r in self.drive if start < r[0] < end})
        dx = dy = dyaw = distance = max_speed = max_turn = 0.0
        for a, b in zip(times, times[1:]):
            va, vb = interpolate(self.drive, a), interpolate(self.drive, b)
            yaw = interpolate(self.rf, (a+b)/2, True)[2]
            vx, vy, wz = [(x+y)/2 for x, y in zip(va, vb)]
            dt = b-a
            dx += (vx*math.cos(yaw)-vy*math.sin(yaw))*dt
            dy += (vx*math.sin(yaw)+vy*math.cos(yaw))*dt
            dyaw += wz*dt
            distance += math.hypot(vx, vy)*dt
            max_speed = max(max_speed, math.hypot(vx,vy))
            max_turn = max(max_turn, abs(wz))
        a, b = interpolate(self.rf, start, True), interpolate(self.rf, end, True)
        mx, my, myaw = b[0]-a[0], b[1]-a[1], angle(b[2]-a[2])
        residual = math.hypot(dx-mx, dy-my)
        # Carrying also closes the wheel gate. Inhibit is harmless when commands
        # are zero, and re-arming waits until the robot has been put down.
        # Allow for telemetry/scan latency and the observed uncertainty of
        # translation during a turn (bounded by the lidar's 0.125 m offset).
        translation_limit = (self.distance_floor + self.relative_error*max(distance, math.hypot(mx,my)) +
                             max_speed*self.timing_tolerance +
                             self.rotation_translation_tolerance*abs(myaw))
        rotation_limit = (self.angle_floor + self.relative_error*max(abs(dyaw),abs(myaw)) +
                          max_turn*self.timing_tolerance)
        bad = residual > translation_limit or abs(angle(dyaw-myaw)) > rotation_limit
        # Moving people can shift scan matching while the base is at rest.
        # Use a separate absolute noise allowance only when drive telemetry was
        # stationary throughout this window. Do not weaken slow-stall detection.
        stationary = max_speed < 0.005 and max_turn < 0.01
        if stationary:
            bad = (residual > self.stationary_distance or
                   abs(angle(dyaw-myaw)) > self.stationary_angle)
        return {'bad': bad, 'end': end, 'residual': residual,
                'measured_distance': math.hypot(mx,my), 'measured_angle': abs(myaw)}


class Recovery:
    """Inhibit acknowledgement and quiet-dwell re-arm policy."""
    def __init__(self, auto_rearm=True, quiet_seconds=3.0):
        self.auto_rearm = auto_rearm
        self.quiet_seconds = quiet_seconds
        self.state = 'WARMUP'
        self.quiet_since = None
        self.manual_reset = False

    def step(self, now, healthy, fault, quiet, ack):
        if fault or not healthy:
            self.state = 'STOPPING'
            self.quiet_since = None
            return True
        if self.state in ('WARMUP', 'NORMAL'):
            if ack is True:
                self.state = 'STALLED'
            else:
                self.state = 'NORMAL'
                return None
        if self.state == 'RESETTING':
            if not quiet:
                self.state = 'STOPPING'
                self.quiet_since = None
                return True
            if ack is False:
                self.state = 'NORMAL'
                self.manual_reset = False
                self.quiet_since = None
                return None
            return False
        if ack is not True:
            self.state = 'STOPPING'
            self.quiet_since = None
            return True
        self.state = 'STALLED'
        if quiet:
            if self.quiet_since is None:
                self.quiet_since = now
            if (self.auto_rearm or self.manual_reset) and now-self.quiet_since >= self.quiet_seconds:
                self.state = 'RESETTING'
                return False
        else:
            self.quiet_since = None
        return True
