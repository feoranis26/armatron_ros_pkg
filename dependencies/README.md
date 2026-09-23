# External ROS dependencies

RF2O remains an external workspace dependency. Import it beside the ARMATRON
packages, then build it before ARMATRON:

```bash
vcs import src < src/armatron_ros_pkg/dependencies/rf2o_laser_odometry.repos
colcon build --symlink-install --packages-select rf2o_laser_odometry
```

The ARMATRON launch uses the Adlink ROS 2 Humble branch and starts its
`rf2o_laser_odometry_node` with TF publication disabled. Record the checked-out
commit with `git -C src/rf2o_laser_odometry rev-parse HEAD` in deployment notes
when promoting a tested build. The upstream manifest may need the local Humble
metadata correction described in the implementation handoff (`nav_msgs` instead
of the obsolete `cmake_modules` dependency); do not vendor that checkout here.
