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

The parameter should be true, and map-to-odom TF should continue arriving with
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
