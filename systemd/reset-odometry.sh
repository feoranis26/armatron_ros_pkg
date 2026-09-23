#!/usr/bin/env bash
# Local x86 diagnostic reset; run with the robot stationary on the floor.
set -euo pipefail

if [[ $# -ne 0 ]]; then
  echo "Usage: $0" >&2
  exit 2
fi

echo "Stopping navigation and teleop before resetting local odometry."
# Navigation's ExecStop saves mapping state while its ROS processes are alive.
sudo systemctl stop armatron-navigation.service armatron-teleop.service
sudo systemctl stop armatron-drive-bridge.service armatron-hardware.service

# Fresh processes clear the bridge position and RF2O/EKF session state.
# The Pi controller is not restarted and its safety inhibit is not reset.
sudo systemctl start armatron-drive-bridge.service armatron-hardware.service

echo "Local odometry processes restarted. Navigation and teleop remain stopped."
echo "Raw drive heading still follows the gyro; RF2O starts at zero heading."
echo "Check /odom/rf2o and /odometry/filtered before resuming."
echo "For manual testing: sudo systemctl start armatron-teleop.service"
