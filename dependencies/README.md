# External ROS dependencies

RF2O remains an external workspace dependency. Import it beside the ARMATRON
packages, then build it before ARMATRON:

```bash
vcs import src < src/armatron_ros_pkg/dependencies/rf2o_laser_odometry.repos
python3 src/armatron_ros_pkg/dependencies/patch_rf2o.py src/rf2o_laser_odometry
colcon build --symlink-install --packages-select rf2o_laser_odometry
```

The ARMATRON launch uses the Adlink ROS 2 Humble branch and starts its
`rf2o_laser_odometry_node` with TF publication disabled. Record the checked-out
commit with `git -C src/rf2o_laser_odometry rev-parse HEAD` in deployment notes
when promoting a tested build. The upstream manifest may need the local Humble
metadata correction described in the implementation handoff (`nav_msgs` instead
of the obsolete `cmake_modules` dependency); do not vendor that checkout here.

The manifest now pins upstream `313bb4c4123bcc0cc2e042f278312b19a3c46f31`.
`patch_rf2o.py` applies `rf2o-confidence.patch` after `git apply --check`, accepts
an already-applied patch, and refuses conflicting source edits. It also handles
the Humble metadata correction and adds `diagnostic_msgs`. The patch publishes
`/rf2o/solver_diagnostics` with each processed scan's timestamp and laser frame.
It does not change the solver or its motion estimates. Rebuild RF2O explicitly;
the ordinary ARMATRON refresh script does not build external packages.

The finest-level robust normal matrix is normalized by point count and scaled
to coordinates `[vx, vy, 2 metres * yaw_rate]`. Its three eigenvalues/eigenvectors,
weighted residual RMS, valid-point count and validity flag are diagnostic
quantities, not calibrated pose covariance. Failed/non-finite solves publish
invalid status rather than reusing previous quality metrics. See
`docs/lidar-confidence.md` for recording, replay and deployment instructions.
