# Heading loss and automatic recovery

Gyro loss temporarily blocks propulsion and filtered scans. It does not set the
Pi safety latch or require an operator reset. Ordinary wheel/LiDAR disagreement
remains diagnostic and does not inhibit propulsion.

The heading guard accepts only fresh, increasing `/imu/gyro` timestamps with a
valid orientation. After one second without a sample it pauses the EKF. Startup
and recovery require 0.3 seconds of fresh observations. A missing guard heartbeat
blocks the drive and scans after 0.3 seconds but does not latch either condition.

Recovery is automatic: acknowledge EKF pause, preserve its current odom pose using
`/set_pose` (clearing stale velocity, acceleration and queued observations), then
acknowledge `/toggle` on before publishing heading readiness. Service failures
keep the gate closed and retry. At first startup no pose reset is necessary if
the EKF has not published a pose yet. `/odometry/heading_status` reports WAITING,
RECOVERING or OK, and state transitions are logged.

The bridge publishes gyro yaw in a continuous local reference, so EKF
`imu0_relative` is false. The first gyro sample establishes zero heading (or the
latest filtered heading available when the bridge starts). After a packet gap
longer than one second, the first returning angle is rebased to the last heading;
subsequent changes are measured normally.

Fresh packets are also checked for implausible angle changes. The bridge rejects
wrapped yaw increments larger than `gyro_max_yaw_rate * dt + gyro_jump_slack`
(defaults 4 rad/s and 0.1 rad), withholds the IMU sample, blocks drive output and
signals the heading guard immediately. Normal 359-to-0 wrapping is accepted.
After at least three coherent samples spanning 0.3 seconds, the new sensor
reference is rebased to the last accepted heading; ordinary automatic EKF
recovery follows. This is a plausibility check, not an independent measurement:
slow drift or a jump within the limit can still pass. Physical rotation during
an outage or rejected interval cannot be reconstructed; verify/reinitialize
global localization if the robot was moved.

Blocked commands and hold-heading targets are discarded, including commands
received during the interruption. Driving requires a new command after readiness
returns. This does not cancel a Nav2 goal: an active controller can issue new
commands after recovery. Explicit `/drive/safety_stop` and external inhibit
requests remain latched; gyro recovery never clears those stops.

No heading fault file controls operation or map saving. The guard removes old
`heading_fault.json` markers on startup when writable. To deploy, rebuild and
restart both the x86 hardware and drive-bridge services with the updated EKF
configuration. A Pi stop already latched by the old implementation is
indistinguishable from an explicit stop: with controls released and heading OK,
clear that legacy stop once using:

```bash
ros2 service call /drive/safety_reset std_srvs/srv/Empty '{}'
```

Subsequent gyro outages need no reset. Do not use this reset to override a separate
intentional drive stop.

Validation on the robot: with navigation stopped and controls released, interrupt
and restore the Pi gyro service. Observe WAITING -> RECOVERING -> OK, held pose
during the outage, no recovery yaw jump and no stale drive command. Verify a new
teleop command works. Separately confirm explicit safety stops survive gyro
recovery. Automated tests cover these control paths; actual ROS service behavior
and hardware timing still need deployment validation.

## Distinguishing gyro resets from SLAM corrections

The Pi service being active proves neither angle accuracy nor localization
accuracy. The x86 drive bridge logs rejected discontinuities and publishes
`/gyro/status` plus `/gyro/heading_valid`. `/gyro/raw_heading_degrees` contains
new raw UDP samples, before sign conversion, unwrapping or reset rebasing.
It is an unstamped Float64; bag receipt time supplies diagnostic timing.

Record the following on x86 before reproducing a jump (stop recording with Ctrl+C):

```bash
ros2 bag record -o heading_jump /gyro/raw_heading_degrees /gyro/heading_valid /gyro/status /imu/gyro /odometry/filtered /odometry/heading_ready /odometry/heading_status /pose /tf /tf_static /scan
```

A raw angle jump implicates the sensor/transport heading stream. An IMU or local
odom jump with a continuous raw angle implicates downstream heading/fusion.
Continuous local odometry with a jumping `map -> odom` points to global SLAM
correction. SLAM does not change the raw gyro reading. This guard does not detect
or reject an incorrect SLAM correction, and it does not cancel a navigation goal.
Mapping mode can add misaligned observations after a localization error; use
localization mode for navigation on an established map to avoid extending its
saved posegraph. That does not itself make localization corrections reliable.
