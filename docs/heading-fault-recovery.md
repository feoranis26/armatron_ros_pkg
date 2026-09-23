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
subsequent changes are measured normally. A sensor-origin change without a packet
gap is not detected. Physical rotation during a missing-data interval cannot be
reconstructed; verify/reinitialize global localization if the robot was moved.

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
