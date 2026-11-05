#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$package_dir/../../tools/environment.sh"
# ROS overlays can expose unrelated Python packages even after conda activate.
runtime="${EMBODIED_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/embodied/bin/python}"
exec env -u PYTHONHOME -u LD_LIBRARY_PATH PYTHONPATH="$package_dir/src" PYTHONNOUSERSITE=1 "$runtime" "$@"
