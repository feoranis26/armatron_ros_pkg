# Navigation TF timing

The global plan and costmap use `map`; the local controller uses `odom`.
The controller therefore needs a current `map -> odom` correction as well as
local `odom -> base_link`. A visually correct SLAM map is not sufficient if
that correction's timestamp stops advancing.

The reported failure had robot pose time 1790220891.878484882 and transform time
1790220890.166032606, a 1.712 s gap, growing to 2.312 s. DWB permits 0.2 s.
It rejected the frame conversion before trajectory scoring and entered recovery.
This is not evidence of a DWB cornering or gyro-angle problem.

Humble slam_toolbox defaults `restamp_tf` to false. Its TF publishing loop can
repeat the same scan timestamp even though it runs at 20 Hz. The navigation
configuration now sets `restamp_tf: true`, `transform_publish_period: 0.05` and
`transform_timeout: 0.5`: publish the latest global correction with current time
plus the existing 0.5 s allowance. Local odometry continues to supply motion
between SLAM corrections. Scan timestamps and DWB tolerance are unchanged.

This preserves availability between corrections; it does not accelerate scan
matching or establish that an old correction is accurate. If SLAM stops
processing scans or accumulates a backlog, investigate that separately. Hardware
heading-loss gates still block drive and scans. No SLAM-progress watchdog is
introduced by this configuration change.

Sources:
- [SLAM TF loop](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp)
- [Humble message filter](https://github.com/ros2/geometry2/blob/humble/tf2_ros/include/tf2_ros/message_filter.hpp)

The message filter labels exceptions from the transform future as `OutTheBack`,
including timeout failures. Its text about messages being earlier than the
cache is therefore not conclusive evidence of an old scan. The explicit
controller data/transform timestamps are more informative here.

## Deploy and check on x86

Save the good map while SLAM is still running before rebuilding/restarting:

```bash
ros2 run armatron armatron-map save
```

Sync the change and rebuild `armatron` using the normal build procedure. Restart
navigation, then confirm the installed configuration:

```bash
sudo systemctl restart armatron-navigation.service
ros2 param get /slam_toolbox restamp_tf
ros2 run tf2_ros tf2_monitor map odom
```

The parameter must be declared and true, and map-to-odom TF should continue arriving with
advancing timestamps while stationary as well as moving. `restamp_tf` is read
at startup in Humble; setting the parameter on the already running node does
not update the member used by its TF loop. A restart is required.

If navigation still fails, retain the complete SLAM and Nav2 log and record:

```bash
ros2 bag record -o navigation_tf /tf /tf_static /scan /pose /odometry/filtered /odometry/heading_ready
```

Compare scan and SLAM pose timestamps while moving; pose publication may pause
when stationary because of minimum-travel thresholds. Increasing lag while
moving suggests processing backlog. Missing SLAM poses despite fresh scans can
also mean SLAM's input TF filter is blocked. Do not infer CPU overload from map
size alone. Check CPU load and SLAM warnings alongside the timestamps.

## Avoiding premature Nav2 recovery failures

`bt_navigator.default_server_timeout` is in milliseconds. The former value 20
allowed only 20 ms for action acknowledgement and service responses. It is now
1000 ms. This does not impose a one-second limit on following an entire path.
Controller `failure_tolerance` is now 1.0 seconds instead of 0.3; on a controller
exception Nav2 sends zero velocity while retrying, then enters recovery if the
failure persists. DWB's TF age tolerance remains 0.2 seconds. These bounded waits
do not repair missing TF, and they do not continue driving on stale transforms.

Localization serves the saved grid through a map server, so SLAM's temporary
scan buffer no longer repeatedly rewrites the global costmap's static layer.
The independent map lifecycle manager activates the map server; the navigation
manager continues to own only the Nav2 nodes. Legacy AMCL and mapping bringup
paths are unchanged. The mapping scan buffer is reduced from 1000 to 10; the
localization override is 3, matching upstream's localization default. This
changes recent scan-matching history, not the number of saved posegraph nodes.

After rebuilding, restart navigation and check:

```bash
ros2 param get /bt_navigator default_server_timeout
ros2 param get /controller_server failure_tolerance
ros2 param get /slam_toolbox map_name
ros2 param get /slam_toolbox scan_buffer_size
ros2 topic info /map --verbose
```

Expect 1000, 1.0, /slam_toolbox/localization_map, 3, and map_server as the sole
/map publisher in localization mode. Robot validation still needs a handoff,
a stationary goal, and a short drive while observing TF and action timeouts.

## Installed-version compatibility and raw timing probe

The current upstream Humble branch implements `restamp_tf`, but tagged releases
2.6.8 and 2.6.10 do not. In those releases the publishing loop always stamps TF
with `scan_header.stamp + transform_timeout`; adding a YAML override does not
implement that feature. A running node reporting `Parameter not set` is not
confirmation that the intended policy is active. Verify the installed package
version with `dpkg-query -W ros-humble-slam-toolbox` before choosing a source
backport or package upgrade. Do not increase TF tolerance to conceal this.

[Tagged 2.6.10 publisher](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp)

The diagnostic tool runs on the x86 host without changing TF or drive state:

```bash
ros2 run armatron tf_timing --duration 20 > tf-timing.json
```

It reports each dynamic TF edge's receipt gaps, message timestamp ages, repeated
and backward timestamps, plus scan, SLAM pose and filtered odometry timing. Run
it during the failure. It also lists discovered TF publisher nodes, but does not
attribute individual edges to those nodes. Its own callback receipt timestamps
are independent of Nav2's buffer and include delivery/scheduling delay.

Fresh local odometry alongside delayed or repeatedly stamped map-to-odom narrows
the issue to global TF publication/delivery. Fresh TF at this probe while Nav2
still reports old TF points toward its listener/executor/cache path. This does
not by itself identify the responsible thread or distinguish every DDS delay
from source-side delay; a bag and node tracing may still be needed.
