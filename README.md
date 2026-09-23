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

The EKF uses drive x/y velocity, RF2O differential planar pose, and BNO heading
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
manual testing. The Pi safety latch is preserved. Raw drive heading still uses
the gyro and does not share RF2O's zero starting heading. Restart localization
and provide an initial pose as needed before resuming navigation after a reset.

`navigation.launch.py` starts RF2O as `/odom/rf2o` with TF disabled, and
`robot_localization` as the sole `odom -> base_link` authority. The stepper
estimate remains available at `/odom/drive_raw` for comparison and for the
motion-consistency monitor. A persistent disagreement latches the Pi drive
inhibit; reset it only after investigating with:

```text
ros2 service call /motion_consistency/reset std_srvs/srv/Empty
```

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
