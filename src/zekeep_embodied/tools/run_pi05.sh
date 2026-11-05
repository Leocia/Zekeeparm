#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
source "$workspace_dir/tools/environment.sh"
# ACT and pi0.5 share the LeRobot 0.4.4 model environment; ROS stays isolated.
runtime="${LEROBOT_PYTHON:-${ACT_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/act/bin/python}}"
export PYTHONPATH="$package_dir/src:$workspace_dir/src/zekeep_act/src"
export PYTHONNOUSERSITE=1
export HF_LEROBOT_HOME="${HF_LEROBOT_HOME:-$workspace_dir/data/lerobot/datasets}"
export ZKEEPI_CHECKPOINTS="${ZKEEPI_CHECKPOINTS:-$workspace_dir/data/lerobot/checkpoints}"
exec "$runtime" "$@"
