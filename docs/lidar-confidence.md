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
reference scan surfaces. All candidates score the same current points;
unmatched points incur a fixed penalty. Fit error, overlap and inlier count
must pass minimum checks. A normalized information matrix identifies weak
directions, with rotation scaled by a 2 m lever arm. Additional positive and
negative 10 cm probes test score ambiguity along the weakest translation
direction. This is an independent diagnostic fit, not RF2O's actual objective
or a replacement scan matcher. Sparse features, repeated geometry, dynamic
objects and partial occlusions can still mislead it.

Native `/rf2o/solver_diagnostics` describes the laser-frame solver matrix.
Independent `/lidar/confidence` information and directions describe the
reference base frame. Neither is covariance; do not feed these matrices to an EKF.

## States

`/lidar/confidence` is JSON in std_msgs/String (schema 1), carrying scan and
reference timestamps, frame, candidate and confirmed state, reason, hypothesis
poses, fit scores, overlap, inliers, information eigenvalues/directions, probe
errors, native solver metrics, and evaluation runtime.

- `CONSISTENT`: drive prediction is compatible with constrained scan evidence.
- `LIDAR_UNDERCONSTRAINED`: weak geometric direction or ambiguous probe scores.
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
the 2.5 cm alignment-score margin is an aggregate fit difference, not a base
displacement limit. No probability or guaranteed stall detection is claimed.

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
