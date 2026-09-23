# Lidar confidence and motion evidence

This stage adds diagnostics without changing RF2O's solution, EKF uncertainty,
propulsion permissions, or the default RF2O+gyro fusion policy. Adaptive
covariance and wheel fallback remain deferred until hallway recordings validate
the diagnostics. A raw drive/RF2O difference is not a confirmed stall.

## Deploy on x86

From the workspace (adjust the ARMATRON checkout name):

```bash
git -C src/rf2o_laser_odometry remote set-url origin https://github.com/feoranis26/rf2o_laser_odometry.git
git -C src/rf2o_laser_odometry fetch origin
git -C src/rf2o_laser_odometry switch --track origin/confidence-diagnostics
bash src/armatron/systemd/build.sh rf2o_laser_odometry
./src/armatron/systemd/refresh.sh x86
ros2 topic echo /lidar/confidence
```

Install `ros-humble-diagnostic-msgs` and `python3-numpy` if missing, or resolve
package dependencies with rosdep. These switch commands are for the first move
to the fork branch; subsequently use `git pull --ff-only` in that checkout.
If earlier patches or other uncommitted edits prevent switching, preserve them
before switching; do not discard them blindly. The pinned dependency manifest
supports fresh checkouts. No Pi update is required.

Without the fork diagnostics, confidence reports `UNAVAILABLE` with an explanation;
the existing odometry path and propulsion continue normally.

## Evidence and frames

`lidar_confidence` compares scans about 0.6 s apart. Static TF maps laser points
to base_link, including the reversed lidar mounting and position offset. It
requires bracketing drive velocity, gyro heading and RF2O poses, rejects gaps
over 0.3 s, never extrapolates, and only evaluates processed RF2O scan stamps
with matching native diagnostics. The IMU must report base_link orientation.
Tilted mounting is rejected; carrying with world roll/pitch is not supported.

Each candidate maps current points into the reference base frame:

- RF2O relative pose.
- Integrated body-frame drive translation rotated using gyro heading change.
- Zero base translation with the same gyro heading change.

The comparison uses bounded point-to-line errors against locally connected
reference scan surfaces. Every candidate uses the same gyro rotation, isolating
translation evidence from yaw disagreement. Scores use the intersection of
matched scan points across all three candidates. Missing overlap and excessive
candidate-dependent loss are assessed separately; unmatched points no longer
add a constant error floor to the hypothesis comparison.

A normalized information matrix measures sensitivity along the drive/RF2O
translation disagreement direction (or travel direction when they agree, or the
weakest direction at rest). The directional strength must be at least 20% of the
strongest translation constraint. A corridor can therefore constrain sideways
motion while leaving longitudinal travel uncertain. Rotation remains scaled by
a 2 m lever arm for the published full matrix, but an unrelated weak rotation
or translation direction no longer automatically rejects all translation.

Contradiction requires paired squared residual improvement exceeding both the
squared score_margin and a conservative three-standard-error bound, with the
effective beam count capped at 20 because neighboring beams are correlated.
This is a heuristic bound, not a calibrated statistical test. RF2O/gyro yaw
disagreement above 0.15 rad is reported as unreliable tracking, not a translation
stall. Repeated geometry, occlusions and dynamic scenes can still mislead it.

Native `/rf2o/solver_diagnostics` describes the laser-frame solver matrix.
Independent `/lidar/confidence` information and directions describe the
reference base frame. Neither is covariance; do not feed these matrices to an EKF.

## States

`/lidar/confidence` is JSON in std_msgs/String (schema 1), carrying scan and
reference timestamps, frame, candidate and confirmed state, reason, hypothesis
poses, fit scores, overlap, inliers, information eigenvalues/directions, tested
translation direction and strength, common overlap, native solver metrics, and evaluation runtime.

- `CONSISTENT`: drive prediction is compatible with constrained scan evidence.
- `LIDAR_UNDERCONSTRAINED`: weak constraints along the tested translation direction.
- `MOTION_CONTRADICTED`: a reliable RF2O fit materially outperforms the drive
  prediction. `near_zero_supported` adds evidence compatible with a stall,
  but also with an incorrect strafe model; it is not a confirmed stall.
- `TRACKING_UNRELIABLE`: insufficient geometry/overlap, invalid native solve,
  poor fit, or RF2O fitting worse than another hypothesis.
- `UNAVAILABLE`: missing/stale inputs, TF, native diagnostics, or warmup.

States require three distinct evaluations and 0.5 s of consecutive evidence.
The candidate state is immediate. Missing evidence clears confirmation. The
monitor mirrors confirmed states and still shows raw estimate disagreement as
a separate field. Optional wheel fusion requires both candidate and confirmed
`CONSISTENT`, no raw disagreement, and its existing agreement dwell. Default
wheel fusion remains off. The confidence node subscribes to /scan and therefore
counts as a consumer for lidar-demand standby.

Initial thresholds live in config/odometry/confidence.yaml. In particular,
the 2.5 cm score_margin is squared when comparing mean squared residuals;
it is neither a base displacement limit nor a difference of aggregate RMS scores. No probability or guaranteed stall detection is claimed.

## Record and replay

Record room motion, a room-to-hallway transition, stationary moving scenery,
strafing, carrying, and a controlled low-speed stall separately. Note approximate
times and actual motion so classifications can be checked against observations.

```bash
ros2 bag record -o hallway_confidence /scan /tf_static /tf /odom/drive_raw /odom/rf2o /imu/gyro /rf2o/solver_diagnostics /lidar/confidence /motion_consistency/status /cmd_vel /odometry/filtered /gyro/status /drive/safety_inhibited
```

The replay tool republishes only recorded sensor/TF inputs in an isolated ROS
domain, drives simulated time, runs the actual confidence node, and saves JSONL.
It requires ROS 2, rosbag2_py and rosgraph_msgs. It refuses the robot domain 67
and default domain 0. Use a domain unused by other robots:

```bash
ROS_DOMAIN_ID=68 python3 src/armatron/tools/replay_lidar_confidence.py hallway_confidence --output hallway-evidence.jsonl
```

The tool requires all diagnostic inputs, including gyro and native RF2O
diagnostics. Older bags missing them cannot validate the complete classifier.
Do not run motor nodes in the replay domain. Default replay rate is real time.

Synthetic tests exercise a corridor, a room, a stationary contradiction,
RF2O underestimation, rotation, missing geometry/overlap, timestamp gaps, and
state persistence. ROS/Linux runtime and real scene validation are still needed
before enabling adaptive covariance. Observe evaluation_ms to assess x86 load.

## Classifier regression results (September 23)

Offline replay reuses recorded scan intervals and motion hypotheses, then
recomputes scores and state dwell from the raw scans. It isolates classification;
it is not a live ROS timing replay. Run it without ROS using Python and NumPy:

```bash
python3 src/armatron/tools/replay_scan_evidence.py hallway_confidence/hallway_confidence_0.db3 --output hallway-revised.jsonl
```

The supplied hallway bag yields 509 confirmed CONSISTENT, 291
LIDAR_UNDERCONSTRAINED and four initial UNAVAILABLE evaluations. No confirmed
motion contradictions occur. The stall bag yields 409 CONSISTENT, 21
MOTION_CONTRADICTED and four initial UNAVAILABLE evaluations. Seven hallway
and six stall evaluations lack a reference scan captured inside the bag and
are skipped. The faster stall is detected; the earlier slow stall is not
reliably detected at the current 0.6-second comparison interval. CONSISTENT
means insufficient evidence to reject the drive prediction, not proof of motion.
The strafing tail produces no sustained contradiction.

These recordings informed the revision and are regression checks, not an
independent validation set. Fusion and propulsion behavior remain unchanged.
A fresh hallway/stall check is still needed before using confidence for fusion.
