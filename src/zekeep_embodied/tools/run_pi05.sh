#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
openpi_dir="$workspace_dir/third_party/openpi"
# Only expose our offline adapters, never ROS overlays or user site-packages.
unset PYTHONHOME LD_LIBRARY_PATH
export PYTHONPATH="$package_dir/src"
export PYTHONNOUSERSITE=1
export HF_LEROBOT_HOME="${HF_LEROBOT_HOME:-$workspace_dir/data/openpi/datasets}"
export ZKEEPI_ASSETS="${ZKEEPI_ASSETS:-$workspace_dir/data/openpi/assets}"
export ZKEEPI_CHECKPOINTS="${ZKEEPI_CHECKPOINTS:-$workspace_dir/data/openpi/checkpoints}"
export XLA_PYTHON_CLIENT_PREALLOCATE=false
exec "$openpi_dir/.venv/bin/python" "$@"
