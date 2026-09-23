# Heading loss fail-safe

Gyro loss is a sensor fault, distinct from ordinary lidar/step disagreement.
The latter remains diagnostic and does not inhibit propulsion.

The bridge blocks commands without fresh gyro packets and a fresh heading-guard
heartbeat. Loss after readiness latches the existing Pi safety stop. Reset requests
are refused while the gyro/guard is unavailable. No automatic motor reset occurs.

The heading guard monitors actual timestamped `/imu/gyro` messages, not repeated
status strings. It latches after one second without a valid new sample, or after
five seconds of startup with no valid sample. Replayed/duplicate timestamps do not
feed it. Scan forwarding waits for readiness and fails closed if guard heartbeats
stop for 0.3 seconds. This gates `/scan` to RF2O and SLAM; `/scan_raw` stays available.

On a latched fault the guard repeatedly requests robot_localization `/toggle`
with `on: false`. Humble stops measurement integration/prediction while publishing
its held state. This does not assert the robot is physically stationary or reset
the stored velocity. Do not toggle it back on and resume an old goal.
If the EKF service is unavailable, scan blocking and drive inhibition still operate
independently, but the guard cannot guarantee the EKF freezes.

The fault is recorded in `/var/lib/armatron/heading_fault.json` (or the configured
ARMATRON_STATE_DIR). While present, map saves and restart-pose writes are refused;
hardware/service restarts alone do not clear it. Previously saved map revisions
remain intact. Some estimator/map error may occur before the one-second timeout;
this protection cannot repair a corrupted session or detect a sensor that keeps
returning fresh but physically wrong values.

## Recovery

1. Stop navigation to discard the active goal. Its automatic save will refuse
   while the fault marker exists; this is intentional. Stop hardware next.
2. Repair/check the gyro connection. Inspect the Pi gyro journal and confirm fresh
   angle packets on x86. Repeated numeric angles while stationary are normal.
3. Keep propulsion inhibited. Preserve any map revision needed for recovery.
   Remove only `/var/lib/armatron/heading_fault.json` after repairing the fault.
   Then start hardware to reset RF2O/EKF/guard. The drive bridge must be running
   to supply IMU samples; start it if needed. Do not use refresh here, since it
   also starts navigation.
4. Confirm `/gyro/status` and `/odometry/heading_status` report OK, and verify
   filtered pose is stable with the robot stationary. Re-establish localization
   against a good saved map (or start a fresh map) before permitting navigation.
5. With old navigation goals gone and controls released, explicitly call
   `/drive/safety_reset`. Start navigation only after localization is trustworthy.

Inspect `journalctl -u armatron-hardware -b --no-pager` on x86 and
`journalctl -u armatron-gyro -b --no-pager` on the Pi.

Validate with propulsion physically prevented: stop the Pi gyro service after
healthy startup; confirm drive inhibit, no `/scan` output, held EKF pose and
map-save refusal. Restart gyro and confirm the fault remains latched. Unit tests
cannot replace this deployment check.
