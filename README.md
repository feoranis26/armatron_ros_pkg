# ARMATRON ROS 2 baseline

This package contains the x86 ROS bridge, sensor bringup, Nav2 launch files,
and the Raspberry Pi gyro publisher.  The motor controller itself is the
separate `armatron_drive` package.

## Runtime topology

The motion-controller Pi runs two independent processes:

```text
ros2 run armatron gyro_pub
taskset -c 3 ros2 run armatron_drive main
```

The nonzero startup velocity commands in `armatron_drive` prepare the waveform
synthesizer before the motor-enable line is asserted. They are not motion
commands. The controller disables its motors if valid UDP motion commands stop
arriving for 500 ms.

The x86 processor runs four responsibilities:

```text
ros2 launch joy_teleop joy_teleop_ubuntu.launch.py
ros2 launch armatron hardware.launch.py
ros2 launch armatron navigation.launch.py slam:=True
ros2 run armatron drive
```

`drive` connects to the Pi at `10.8.3.56` by default. Override the
`controller_host`, port, or local-listen-port ROS parameters when needed.

## Navigation and visualization

The default navigation mode has lateral velocity disabled because the mecanum
base is presently unreliable. When it is healthy, enable strafing without
switching parameter files:

```text
ros2 launch armatron navigation.launch.py slam:=True holonomic:=true
```

Start the committed RViz layout with:

```text
ros2 launch armatron visualization.launch.py
```

## Fused odometry and map profiles

The EKF defaults to RF2O differential planar pose and BNO heading
on `/imu/gyro`. Wheel-derived yaw rate is excluded. The x86 bridge adapts the
existing UDP Euler heading into a yaw-only IMU message; angular velocity and
acceleration are marked unavailable. Heading is fused relative to the first
sample after EKF startup, not differentiated. Only fresh, new packets produce
IMU messages, timestamped at bridge publication (the UDP protocol has no sensor
timestamp). `gyro_yaw_variance` on the drive bridge defaults to 0.0025 rad² as
an initial uncertainty assumption, not a calibrated accuracy specification.
The heading conversion matches the existing planar body convention validated
in the turn recording; full 3D carrying and magnetic disturbances are not
validated. Add `/imu/gyro` to diagnostic bag recordings.

Gyro transport health is published by the x86 bridge on `/gyro/status`. Missing
or invalid angle packets for one second produce STALE status and a journal
error every five seconds; raw pose heading is held at its last value (zero
before the first reading). The Pi gyro service fails if either worker exits or
five seconds pass without a valid sensor reading, including before a client
connects. Its existing `Restart=no` policy leaves failures visible for diagnosis.

For a fresh local odometry session during diagnosis, place the robot stationary
on the floor and run `bash ./src/armatron/systemd/reset-odometry.sh` on the x86
(adjust the checkout directory name if needed). This stops navigation and
teleop, then restarts the drive bridge and hardware group, including RF2O and
the EKF. Navigation and teleop stay stopped; explicitly start teleop to resume
manual testing. The script does not reset the Pi latch; explicitly reset it with `/drive/safety_reset` if needed. Raw drive heading still uses
the gyro and does not share RF2O's zero starting heading. Restart localization
and provide an initial pose as needed before resuming navigation after a reset.

`hardware.launch.py` starts RF2O as `/odom/rf2o` with TF disabled, and
`robot_localization` as the sole `odom -> base_link` authority. The stepper
estimate remains available at `/odom/drive_raw` for comparison and for the
motion-consistency monitor. `/odom/rf2o_fusion` supplies lidar measurements with
explicit covariance. Wheel translation is disabled by default; wheel rotation
is never fused. Raw topics remain available for diagnostics.

The monitor compares integrated body velocities against RF2O displacement over
matching 0.75 s and 2.5 s windows. It reports `CONSISTENT`, `DISAGREEMENT`, or
`UNAVAILABLE` on `/motion_consistency/status`. Disagreement, missing sensors,
and loss of the monitor do not inhibit propulsion. A corridor, slipping wheels,
a stall, carrying, and moving scenery can produce indistinguishable disagreement.
This monitor does not attempt collision recovery or automatically reset a stop.
SLAM may correct drift but shares the lidar's geometric limitations.

Settings are in `config/odometry/monitor.yaml`. `use_drive_fusion: false` is the
default. If explicitly enabled, `/odom/drive_validated` supplies wheel x/y
velocity only after three seconds of continuous healthy agreement; disagreement
or stale data immediately closes that fusion gate. `agreement_seconds` controls
this delay. `/drive/odometry_valid` reports whether wheel fusion is enabled and
allowed; false is normal in the default configuration and is not a drive stop.
The stationary comparison allowances remain configurable diagnostic thresholds.

Explicit `/drive/safety_stop` and `/drive/safety_reset` services and the
`/drive/inhibit_request` topic remain available to a separate supervisor.
`/drive/safety_inhibited` reports fresh Pi feedback. The bridge still clamps
commands while Pi feedback is stale or the Pi latch is set, retries explicit
stop/reset requests, and zeros commands after one second without `/cmd_vel`.
The Pi's independent 500 ms UDP command timeout remains unchanged.

Deploy with the x86 refresh script; no Pi code update is required for this change.
An old latched stop is intentionally not cleared automatically. With motion
commands released, clear it once if necessary:

```text
ros2 service call /drive/safety_reset std_srvs/srv/Empty '{}'
```

The old `/motion_consistency/reset` service is removed. Verify that disagreement
changes status without setting the Pi latch, `/odom/rf2o_fusion` keeps publishing,
and `/odom/drive_validated` stays silent by default. Test explicit stop/reset and
command timeout separately. RF2O sensor loss is reported but does not stop teleop;
autonomous navigation must handle localization loss through its own supervision.

Navigation requires an explicit map profile. Runtime state defaults to
`/var/lib/armatron` under the systemd service and can be redirected with
`ARMATRON_STATE_DIR` for developer testing:

```text
armatron-map new primary
armatron-map select primary --mode mapping
# drive and map, then stop navigation cleanly to serialize a revision
armatron-map set-mode localization
armatron-map status
```

Mapping saves are staged and retain the prior revision. Localization mode never
updates the selected posegraph. RF2O source remains external; see
`dependencies/README.md` before building on a new x86 workspace.

## systemd installation

### Lidar idle standby

Hardware bringup includes `lidar_demand`. It checks the ROS graph every 0.5 s
and calls the RPLidar `/stop_motor` service after 5 s without consumers of
`/scan` or `/scan_raw`. The internal `/scan_filter` raw subscription is ignored.
Any consumer returning triggers `/start_motor`; the driver stays running so
it can receive that request. Discovery and motor spin-up add some wake latency.
RF2O counts as a consumer, so normal local odometry keeps the lidar running even
when navigation and RViz are closed. Topic recorders and `ros2 topic hz` also
count. This is subscriber-based standby, not a robot inactivity timer.

The watcher logs consumer changes, service completion, and throttled failures
to `journalctl -u armatron-hardware`. Empty service responses acknowledge the
request, not physical motor health. It retries unavailable/timed-out services
and reapplies demand when the raw-scan publisher restarts. Keep the driver's
`auto_standby` disabled; the launch file sets this explicitly.

For standalone use: `ros2 run armatron lidar_demand`. Parameters are
`idle_seconds`, `raw_topic`, `scan_topic`, `filter_node` (fully qualified),
`start_service`, and `stop_service`. Run only one watcher per lidar.
To test standby, stop scan consumers (including RF2O), leave the driver,
filter, and watcher running, and wait five seconds. Subscribing to `/scan`
should wake it. Closing that subscriber should stop it again.

### Installing services

All service units explicitly set `ROS_DOMAIN_ID=67`. Set the same value in
interactive terminals before running RViz, ROS CLI tools, or manual launches:
`export ROS_DOMAIN_ID=67`.

Unit files are source-controlled under `systemd/pi` and `systemd/x86`.
Link the selected host's checked-out units with:

```text
sudo ./systemd/install.sh pi
sudo ./systemd/install.sh x86
```

For a normal code/unit-file refresh, build as `feoranis` and let the helper use
`sudo` only for systemd work:

```text
./src/armatron_ros_pkg/systemd/refresh.sh pi
./src/armatron_ros_pkg/systemd/refresh.sh x86
```

The helper clears only `build/armatron` and `install/armatron` before building.
This avoids stale setuptools manifests and symlinks retaining a deleted launch
file such as `amcl.launch.py`.

The linker refuses to replace a regular file, reloads systemd, and does not
enable or start any unit. Run `systemd-analyze verify` before enabling a target.
The units intentionally use
`Restart=no`; a failed process stays failed for diagnosis with `systemctl
status` and `journalctl -u`.

The service paths assume `/home/feoranis/dev_ws`. Update
`ARMATRON_WORKSPACE`, `User`, and `ExecStart` if the deployment path differs.

The gyro service runs as `feoranis`, using that user's Python packages
(`adafruit-blinka` and `adafruit-circuitpython-bno055`). That user must have
read/write access to the Pi's I2C device. Check `ls -l /dev/i2c-*` and
`id feoranis`; if the device belongs to group `i2c`, grant access with
`sudo usermod -aG i2c feoranis`, then restart the gyro service. The separate
motor-controller service retains its root user for pigpio.

After updating a linked unit file, run `sudo systemctl daemon-reload` and
restart the affected service. Unit-only changes do not require a colcon build.
