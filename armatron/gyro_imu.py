"""Adapt fresh BNO heading packets to a planar IMU observation."""
import math


class GyroImuAdapter:
    def __init__(self, yaw_variance, message_type):
        if not math.isfinite(yaw_variance) or yaw_variance <= 0:
            raise ValueError('gyro_yaw_variance must be positive and finite')
        self.yaw_variance = yaw_variance
        self.message_type = message_type
        self.last_sample_time = None

    def message(self, sample, now, stamp, frame):
        if sample is None:
            return None
        degrees, received = sample
        if (not math.isfinite(degrees) or now - received > 1.0 or
                received == self.last_sample_time):
            return None
        self.last_sample_time = received
        yaw = -math.radians(degrees)  # Same body-yaw convention as the bridge.
        msg = self.message_type()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.orientation.z = math.sin(yaw / 2)
        msg.orientation.w = math.cos(yaw / 2)
        # Only yaw is measured. The EKF explicitly excludes roll and pitch.
        msg.orientation_covariance = [1e6, 0., 0., 0., 1e6, 0., 0., 0., self.yaw_variance]
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        return msg
