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
ros2 launch armatron navigation.launch.py slam:=true
ros2 run armatron drive
```

`drive` connects to the Pi at `10.8.3.56` by default. Override the
`controller_host`, port, or local-listen-port ROS parameters when needed.

## Navigation and visualization

The default navigation mode has lateral velocity disabled because the mecanum
base is presently unreliable. When it is healthy, enable strafing without
switching parameter files:

```text
ros2 launch armatron navigation.launch.py slam:=true holonomic:=true
```

Start the committed RViz layout with:

```text
ros2 launch armatron visualization.launch.py
```

## Posegraphs

`armatron-posegraph` stores files in
`~/.local/share/armatron/posegraphs` by default (override with
`ARMATRON_POSEGRAPH_DIR`). It uses the standard `slam_toolbox` serialization
services, so `slam_toolbox` must already be running.

```text
armatron-posegraph save lab1
armatron-posegraph load lab1
armatron-posegraph list
```

Saving an existing name requires `--force`.
Loading starts the restored graph at the current origin. Use slam_toolbox's
normal localization launch parameters when it needs a different initial pose.

## systemd installation

Unit files are source-controlled under `systemd/pi` and `systemd/x86`.
Link the selected host's checked-out units with:

```text
sudo ./systemd/install.sh pi
sudo ./systemd/install.sh x86
```

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
