#!/usr/bin/env bash
# Build this package as the developer, then update and restart its services.
set -euo pipefail

if [[ $# -ne 1 || ( $1 != pi && $1 != x86 ) ]]; then
  echo "Usage: $0 <pi|x86>" >&2
  exit 2
fi

host=$1
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "${script_dir}/../../.." && pwd)

cd "${workspace}"
# setuptools records package data in build output and symlink-install retains
# its old launch-file links in the package-local install tree. Both are
# disposable package outputs; leave every other workspace package untouched.
rm -rf "${workspace}/build/armatron" "${workspace}/install/armatron"
colcon build --symlink-install --packages-select armatron
sudo "${script_dir}/install.sh" "${host}"

if [[ ${host} == pi ]]; then
  sudo systemctl restart armatron-gyro.service
else
  sudo systemctl restart \
    armatron-hardware.service \
    armatron-drive-bridge.service \
    armatron-teleop.service \
    armatron-navigation.service
fi
