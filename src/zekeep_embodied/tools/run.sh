#!/usr/bin/env bash
set -euo pipefail
# ROS overlays can expose unrelated Python packages even after conda activate.
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ZKEEP_VISION_PYTHON="${ZKEEP_VISION_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm/bin/python}"
runtime="${EMBODIED_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/embodied/bin/python}"
exec env -u PYTHONHOME -u LD_LIBRARY_PATH PYTHONPATH="$package_dir/src" PYTHONNOUSERSITE=1 "$runtime" "$@"
