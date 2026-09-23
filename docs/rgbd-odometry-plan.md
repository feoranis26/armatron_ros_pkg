# RGB-D odometry implementation plan

Status: proposed; camera generation and working ROS driver still need confirmation.

## Scope

Add Kinect visual odometry as independent translation evidence, especially along
hallways where 2D lidar is weak. Keep slam_toolbox, map profiles and Nav2. Run only
RTAB-Map's standalone `rtabmap_odom/rgbd_odometry` component: no RTAB-Map mapping
database, loop closure, visualization application or replacement navigation stack.
Keep gyro heading and the EKF as the sole odom-to-base_link TF publisher.
Odometry disagreement must never inhibit propulsion.

## 1. Establish the camera contract first

Identify Kinect generation, driver/version, USB connection and x86 resource budget.
Verify actual RGB, registered depth and CameraInfo topics before writing remaps.
Record image sizes, encodings, rates, timestamps and optical frame names. Check
depth scale, camera intrinsics and RGB/depth registration on visible edges.
Verify the calibrated base_link-to-camera optical transform, including orientation.
Do not substitute guessed calibration or use unregistered depth with RGB intrinsics.

Deliver a short camera-only launch/check command and a stationary/translation/turn
recording. Camera acquisition must work independently of RTAB-Map. Camera driver
selection remains unresolved until the Kinect model is known.

## 2. Make standalone visual odometry observable

Use a Humble-compatible RTAB-Map package, recording the tested version. Add an
optional `rgbd_odometry.launch.py` and one configuration file. Start with direct
RGB/depth/CameraInfo subscriptions; add a separate sync node only if required.
Choose exact or bounded approximate synchronization from measured camera timing.

Remap the output to `/odom/rgbd`; expose tracking diagnostics on a separate topic.
Disable odometry TF publication. Retain private visual-odometry origin/frame names
so the camera cannot compete with EKF TF. Initially run without wheel/EKF pose
guesses or gyro assistance to measure independent tracking quality.

Expose missing streams, synchronization failures and tracking loss clearly. A
stale pose, restart or tracking failure is not a zero-velocity observation.
Start manually with one documented command; do not enable a service yet.

## 3. Validate before fusion

Record synchronized camera inputs/calibration, `/odom/rgbd`, tracking diagnostics,
RF2O/raw drive odometry, gyro, lidar confidence, filtered odometry and static TF.
Run room translation/rotation, strafing, a hallway transition, hand-carried motion,
a controlled stall, occlusion and camera unplug/restart. Annotate actual motion.

Check sign, scale, timestamps, base-frame conversion, tracking continuity and x86
CPU/latency. Compare displacement against marked distances. Establish measurable
acceptance bounds from the navigation requirements before accepting the result.
Do not treat another estimator's trajectory as ground truth.

## 4. Integrate translation with explicit quality handling

Adapt visual odometry into body-frame translation velocity with covariance. Confirm
the upstream twist frame and semantics before using it; otherwise derive velocity
from timestamped poses. Include camera lever-arm effects through the base transform.
Discard discontinuities and reset the differentiator after tracking loss/restart.
Keep camera yaw out of the EKF measurement selection initially.

Extend the existing fusion adapter rather than introducing another estimator chain:

- Healthy camera translation remains available when RF2O is underconstrained.
- RF2O retains its directional confidence weighting.
- Step fallback is allowed only in directions lacking trustworthy camera AND lidar
  evidence. A healthy camera observing zero movement must override invented step
  motion during a stall.
- Missing/invalid camera data withdraws that measurement; it never publishes a
  fabricated stationary observation. Stale confidence cannot restore wheel trust.

Use conservative bounded covariances informed by tracking quality and recordings.
Feature count alone is not a calibrated uncertainty estimate. Test conflicting
camera/lidar data and avoid overconfident fusion of correlated information; do not
feed fused odometry back into visual tracking in this initial implementation.
Publish fusion status explaining each input's eligibility and active fallback.

## 5. Deployment and rollback

Add an optional x86 camera/visual-odometry service after standalone validation.
Camera failure must not terminate lidar, drive or gyro services. Make refresh
behavior explicit, with camera support disabled by default until verified on this
robot. Provide dependency checks and actionable errors before launch.

One configuration switch must restore the current lidar/gyro/step policy. Keep map
profiles and existing navigation commands unchanged. Document restart behavior so
a visual-odometry origin reset cannot become an EKF velocity spike.

## Verification

Unit-test timestamp ordering, frame/lever-arm conversion, reset rejection, quality
expiry and arbitration when camera says stationary but steps report movement.
Use ROS replay for synchronization and EKF integration, then repeat the live test
sequence. Accept only after failure/restart tests and the hallway stall test pass.
Do not install a broad RTAB-Map launch stack merely to obtain one odometry node.

## References

- [RTAB-Map ROS packages and odometry components](https://github.com/introlab/rtabmap_ros)
- [RGB-D odometry subscriptions and synchronization](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/nodelets/rgbd_odometry.cpp)

These references describe upstream behavior; exact Humble package parameters must
be verified against the installed version during implementation.
