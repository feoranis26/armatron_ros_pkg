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
# setuptools records package data in this directory. Clearing only this
# disposable build output prevents deleted launch files from surviving in its
# manifest while retaining all other workspace build products.
rm -rf "${workspace}/build/armatron"
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
