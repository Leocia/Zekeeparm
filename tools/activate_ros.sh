# Source from any directory: source /path/to/workspace/tools/activate_ros.sh
zekeep_workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$zekeep_workspace_dir/tools/environment.sh"
zekeep_venv_dir="${ZKEEP_ROS_VENV:-$zekeep_workspace_dir/.venv-ros}"
if [[ ! -f "$zekeep_venv_dir/conda-meta/history" || ! -x "$zekeep_venv_dir/bin/python" ]]; then
    echo "Create the ROS Conda environment first; see docs/SDK_INSTALL.md" >&2
    return 1
fi
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$ZKEEP_MINIFORGE_DIR/etc/profile.d/conda.sh"
conda activate "$zekeep_venv_dir"
# Preserve the environment-root field used by existing ROS launch wrappers.
export VIRTUAL_ENV="$zekeep_venv_dir"
export ZKEEP_ROS_VENV="$zekeep_venv_dir"
if [[ -f "$zekeep_workspace_dir/install/setup.bash" ]]; then
    source "$zekeep_workspace_dir/install/setup.bash"
fi
zekeep_python_site="$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
# Keep Conda Pinocchio ahead of ROS's older bindings without loading apt Python packages.
export PYTHONPATH="$zekeep_python_site:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$zekeep_venv_dir/lib:${LD_LIBRARY_PATH:-}"
