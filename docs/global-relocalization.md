# AMCL bootstrap into slam_toolbox — stage 1

`armatron-map relocalize` is an operator-assisted bootstrap, not another permanent
localizer. It makes no motion commands and leaves RF2O, the gyro bridge and EKF
running. Automatic convergence decisions, active probes, startup fallback and
uninterrupted Nav2 lifecycle integration remain deferred until this transfer is
validated on the robot. Dock machinery is not involved.

## Run on x86

Sync/rebuild the package. If the profile does not already have a grid, while its
mapping session is running save the paired artifacts:

```bash
ros2 run armatron armatron-map save --with-grid
```

Then stop normal navigation (also stop any manually launched copy). Leave hardware,
drive bridge and odometry services running:

```bash
sudo systemctl stop armatron-navigation.service
ros2 run armatron armatron-map relocalize
```

The command refuses existing navigation/localization nodes and observed map->odom
publishers. It copies the selected revision into its run directory so both AMCL
and SLAM use the same snapshot. The original profile is not modified during search.
No need to switch the profile to `amcl` mode.

View the map, scan, AMCL pose and particle cloud in the updated navigation RViz
configuration, with fixed frame `map`. Global particles should settle at the
physical location, with scan endpoints aligned to walls. Inspect more than a
single plausible scan: repeated corridor geometry can still produce a wrong
hypothesis. You may use supervised teleop during search. This command does not
provide autonomous collision-checked probes.

AMCL receives a temporary startup seed only to confirm it has consumed its map
and a scan. The command then requests a uniform global reset, discards that seed,
and begins operator acceptance. No-motion updates are requested once per second
so an estimate can remain fresh after teleop stops. Repeated stationary scans
are not treated as proof of convergence.

Release controls, let the robot remain stationary, and accept in another sourced
terminal only when the estimate agrees with its physical location:

```bash
ros2 service call /armatron/relocalize/accept std_srvs/srv/Trigger '{}'
```

This is a stage-one manual convergence decision. A `success: true` acceptance
response means handoff has begun, not that handoff succeeded. Watch the original
command for its final result. It refuses acceptance without a fresh map pose,
fresh heading/scans/filtered odometry, and at least one second stationary.

The command deactivates AMCL and waits for its process group to exit before
starting slam_toolbox localization. It also stops the temporary map server and
clears cached AMCL TF. It seeds the loaded posegraph through `/initialpose`, then
requires at least three distinct subsequent SLAM poses spanning one second,
plus TF agreement, while the robot remains stationary. Default discrepancy limits
are 0.3 m and 0.25 rad; these are handoff checks, not claims of localization accuracy.

After `VERIFIED` and successful command exit, the recovered SLAM pose is saved as
`last_pose` and the profile mode becomes `localization`. The temporary SLAM process
is stopped. Keep the robot stationary and start normal navigation:

```bash
sudo systemctl start armatron-navigation.service
```

Normal slam_toolbox now loads that pose. Verify alignment before sending a goal.
This first stage deliberately has a short no-map-TF interval between the bootstrap
ending and normal navigation starting. Do not reset local odometry or move the
robot during that interval. Seamless lifecycle takeover comes in a later stage.

## Failure and diagnostics

Search times out after 300 seconds; `--timeout` changes this. `--distance-tolerance`
and `--angle-tolerance` configure handoff checks. An incorrect SLAM result, gyro
loss, movement during handoff, process failure, duplicate TF publishers or service
timeout fails the command; no recovered pose is persisted. A profile/mode/revision
change during the run also prevents persistence. Ctrl+C cleans up owned processes.
The command does not restart navigation on success or failure and cannot prevent
someone independently starting another launch; do not start one during recovery.

Run snapshots, generated parameters, process logs and status logs are retained
under `/var/lib/armatron/relocalization_runs/<id>/` (or `--state-dir`). A cleanup
failure reports an error: resolve surviving processes before starting navigation.

Tests simulate lifecycle replies, process ownership/order, valid and invalid
handoffs, and rejection of an existing navigation session. They do not replace a
Humble runtime test. First verify a known pose, then repeat from a different known
location; check scan alignment and continuity of map->base_link across handoff.

The legacy standalone `amcl` mode and `global-localize` remain available for
manual diagnostics. They do not perform this SLAM handoff.
