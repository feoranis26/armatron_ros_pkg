#!/usr/bin/env bash
# Build without inheriting the workspace overlay from the interactive shell.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "${script_dir}/../../.." && pwd)
ros_setup=${ARMATRON_ROS_SETUP:-/opt/ros/humble/setup.bash}
if [[ ! -f ${ros_setup} ]]; then
  echo "ROS underlay setup not found: ${ros_setup}" >&2
  exit 1
fi
if [[ $# -eq 0 ]]; then
  set -- armatron
fi

cd "${workspace}"
# Clearing only AMENT_PREFIX_PATH leaves stale CMake/Python/library paths.
# Start fresh and source the underlay, without reading shell startup files.
# Preserve the user's home so user-installed colcon and its config still work.
env -i HOME="${HOME}" USER="$(id -un)" LOGNAME="$(id -un)" \
  LANG="${LANG:-C.UTF-8}" \
  PATH="${HOME}/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  bash --noprofile --norc -c '
    set -eo pipefail
    source "$1"
    shift
    exec colcon build --symlink-install --packages-select "$@"
  ' bash "${ros_setup}" "$@"
