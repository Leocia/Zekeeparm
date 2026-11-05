#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
source "$workspace_dir/tools/environment.sh"
runtime="${EMBODIED_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/embodied/bin/python}"

# Keep ROS message packages while avoiding Python packages from unrelated overlays.
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
export PYTHONPATH="$package_dir/src:$workspace_dir/src/zekeep_act/src:$workspace_dir/src/zekeep_teleop/src:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
exec "$runtime" "$@"
