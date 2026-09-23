# RF2O source repository

RF2O lives beside the ARMATRON packages in its own source checkout. We use
https://github.com/feoranis26/rf2o_laser_odometry, branch `confidence-diagnostics`.
The workspace manifest pins commit `4a0c245bdc00c5757f4bcee16fd8598f8c06e26f`
for reproducible fresh imports:

```bash
vcs import src < src/armatron_ros_pkg/dependencies/rf2o_laser_odometry.repos
bash src/armatron_ros_pkg/systemd/build.sh rf2o_laser_odometry
```

The fork contains the diagnostics and Humble dependency metadata corrections;
no patch installer is needed. Existing checkouts should fetch the fork and
switch to `confidence-diagnostics`. Preserve any local edits when switching.
See [deployment instructions](../docs/lidar-confidence.md).

The ARMATRON launch starts `rf2o_laser_odometry_node` with TF disabled.
`/rf2o/solver_diagnostics` carries each processed scan's timestamp and laser
frame. The diagnostics do not change the solver's motion estimates. Rebuild
RF2O explicitly; ARMATRON's refresh script builds only the armatron package.

The finest-level robust normal matrix is normalized by point count and scaled
to coordinates `[vx, vy, 2 metres * yaw_rate]`. Its three eigenvalues/eigenvectors,
weighted residual RMS, valid-point count and validity flag are diagnostic
quantities, not calibrated pose covariance. Failed/non-finite solves publish
invalid status rather than reusing previous quality metrics. See
`docs/lidar-confidence.md` for recording, replay and deployment instructions.
