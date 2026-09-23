"""Validation helpers for operator-assisted global localization."""
import math


def planar_pose(message):
    if message.header.frame_id != 'map':
        raise ValueError('Expected a pose in map')
    p, q = message.pose.pose.position, message.pose.pose.orientation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite localization pose')
    if abs(sum(v*v for v in values[2:])-1.) > .01:
        raise ValueError('Invalid localization quaternion')
    return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def nearby(a, b, distance, angle):
    return (math.hypot(a[0]-b[0], a[1]-b[1]) <= distance and
            abs(math.atan2(math.sin(a[2]-b[2]), math.cos(a[2]-b[2]))) <= angle)


def stationary(odom, linear=.02, angular=.03):
    t = odom.twist.twist
    values = (t.linear.x, t.linear.y, t.angular.z)
    return (all(math.isfinite(v) for v in values) and
            math.hypot(*values[:2]) <= linear and abs(values[2]) <= angular)
