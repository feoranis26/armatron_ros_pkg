# Costmap noise, self-return masks and AMCL

## Costmap denoising

Both costmaps run Humble's `nav2_costmap_2d::DenoiseLayer` after obstacle marking
and before inflation. `minimal_group_size: 2` removes isolated occupied cells;
8-connectivity retains diagonally adjacent cells. This is spatial filtering, not
multi-scan confirmation. A genuine object represented by one isolated cell may
also disappear, while noise next to an existing obstacle may survive. Test narrow
obstacles explicitly. The filter also sees static-map cells in the global costmap.
Set `denoise_layer.enabled: false` in both costmaps to disable it.

Before deployment on an older Humble installation, verify the plugin is installed:

```bash
grep -R DenoiseLayer /opt/ros/humble/share/nav2_costmap_2d
```

The filter does not alter `/scan` or the RF2O/SLAM observations.

## Self-return geometry

`config/odometry/scan_filter.yaml` replaces the old 0.35 m exclusion circle.
The scan's original `range_min`, intensities and timing are preserved. Endpoints
inside configured rectangles become NaN (unknown), not a free-space ray.
Nearby endpoints outside those rectangles remain valid. The sensor's own minimum
range still applies. AMCL now also uses that sensor minimum (`laser_min_range: -1`).

`profile_boxes` is a JSON list, stored as a YAML string, of
`[center_x, center_y, width, height, yaw_radians]` in metres/radians.
`mask_frame: ""` uses the incoming scan frame; a named frame such as `base_link`
uses the timestamped TF transform. Missing/non-planar TF forwards an unmasked
scan and logs an error rather than masking guessed locations. Default padding
is 5 mm and must be checked against the actual returns.

Measured geometry: 20 mm square posts with a 340 mm outside-to-outside square
have centres at +/-160 mm. The lidar is offset 117 mm toward robot front.
The supplied hallway recording has `base_link -> laser_frame` rotated nearly
180 degrees, so the front/near pair is at laser X = -0.043 m, and the rear/far
pair is at laser X = +0.277 m, each with Y = +/-0.160 m. These four 20 mm boxes
are configured in laser_frame. Verify the current mount still matches that recording.
The recorded translation was 0.125 m; masks use the supplied measured 0.117 m
relative geometry rather than inferring it from that older translation.

Compare `/scan_raw` and `/scan` with a nearby obstacle before navigating; only
profile endpoints should disappear. If the physical mount or scan orientation
changes, update this configuration. The 5 mm padding is an initial tolerance.

## AMCL profile mode

Mapping and existing slam_toolbox localization modes remain available. The new
`amcl` mode launches Nav2 with map_server + AMCL and forces SLAM off, including
when the systemd unit supplies the legacy slam argument. Only AMCL then publishes
`map -> odom`; the EKF continues to publish `odom -> base_link`.

First export a grid while the mapping session is running:

```bash
ros2 run armatron armatron-map save --with-grid
```

This stages the posegraph plus `grid/map.yaml` and `grid/map.pgm`, then promotes
the revision only if both operations succeed. Once a grid exists, subsequent
mapping saves (including systemd stop) regenerate it. A grid-export failure
preserves the prior revision. Both snapshots are taken sequentially from the
live mapper; save while stationary for the closest correspondence.

After saving, stop the mapping navigation instance before changing mode:

```bash
sudo systemctl stop armatron-navigation.service
ros2 run armatron armatron-map set-mode amcl
sudo systemctl start armatron-navigation.service
ros2 lifecycle get /amcl
```

For manual launches, terminate the mapping launch instead of stopping its service,
then use `ros2 launch armatron navigation.launch.py` after selecting AMCL mode.
Do not run service and manual navigation together. A saved map-frame pose is an
initial hint when available; RViz 2D Pose Estimate can provide a better one.

For global localization, with no active navigation goal:

```bash
ros2 run armatron armatron-map global-localize
```

This requests `/reinitialize_global_localization`. It spreads particles across
the map; it neither guarantees convergence nor commands recovery motion. Observe
map/scan alignment and AMCL pose/particle distribution before sending goals.
Repeated corridors can remain ambiguous; a manual initial pose may be needed.
Any controlled motion to gather evidence is an operator decision in this stage.
No automatic goal resumption or gyro-fault reset is added.

AMCL pose persistence subscribes to `/amcl_pose`. Returning to `mapping` or
`localization` uses slam_toolbox and its posegraph again; selecting a mode alone
does not change an already-running process. Restart navigation after mode changes.

## Validation limits

Local tests cover endpoint geometry, close returns, transform arithmetic, layer
order, grid staging and preservation, and launch selection. ROS plugin loading,
camera-free global localization performance, and physical collision behavior
still need validation on the x86/robot.

References: [Humble denoise implementation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/denoise_layer.cpp),
[AMCL configuration](https://docs.nav2.org/configuration/packages/configuring-amcl.html).
