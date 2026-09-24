"""Adapt fresh BNO heading packets to a planar IMU observation."""
import math


class GyroImuAdapter:
    def __init__(self, yaw_variance, message_type, max_yaw_rate=4.0, jump_slack=0.1):
        if not math.isfinite(yaw_variance) or yaw_variance <= 0:
            raise ValueError('gyro_yaw_variance must be positive and finite')
        if not all(math.isfinite(v) and v > 0 for v in (max_yaw_rate, jump_slack)):
            raise ValueError('gyro plausibility limits must be positive and finite')
        self.max_yaw_rate, self.jump_slack = max_yaw_rate, jump_slack
        self.tracking_valid = False
        self.last_raw = None
        self.candidate = None
        self.rejections = 0
        self.yaw_variance = yaw_variance
        self.message_type = message_type
        self.last_sample_time = None
        self.yaw = None
        self.offset = 0.0
        self.seed_yaw = 0.0

    def message(self, sample, now, stamp, frame):
        if sample is None:
            self.tracking_valid = False
            return None
        degrees, received = sample
        if not math.isfinite(degrees) or now-received > 1.0 or received > now:
            self.tracking_valid = False
            return None
        if self.last_sample_time is not None and received <= self.last_sample_time:
            return None
        raw_yaw = -math.radians(degrees)
        dt = received-self.last_sample_time if self.last_sample_time is not None else None
        delta = (math.atan2(math.sin(raw_yaw-self.last_raw), math.cos(raw_yaw-self.last_raw))
                 if self.last_raw is not None else 0.)
        if dt is None:
            self.offset = self.seed_yaw-raw_yaw
        elif dt > 1.0:
            self.offset = self.yaw-raw_yaw
            self.candidate = None
        elif abs(delta) > self.max_yaw_rate*dt+self.jump_slack or self.candidate is not None:
            self.tracking_valid = False
            if self.candidate is None:
                self.rejections += 1
                self.candidate = (raw_yaw, received, 1)
            else:
                previous, since, count = self.candidate
                step = abs(math.atan2(math.sin(raw_yaw-previous), math.cos(raw_yaw-previous)))
                if step > self.max_yaw_rate*dt+self.jump_slack:
                    self.candidate = (raw_yaw, received, 1)
                else:
                    self.candidate = (raw_yaw, since, count+1)
            self.last_sample_time = received
            previous, since, count = self.candidate
            if received-since < .3 or count < 3:
                return None
            # A persistent new reference is not physical rotation. Rebase it
            # only after a coherent stream, without feeding the jump to the EKF.
            self.offset = self.yaw-raw_yaw
            self.candidate = None
        else:
            # Unwrap 359 -> 0 degrees without a spurious full revolution.
            self.offset = self.yaw+delta-raw_yaw
        self.last_raw = raw_yaw
        self.tracking_valid = True
        self.last_sample_time = received
        yaw = raw_yaw + self.offset
        self.yaw = yaw
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
