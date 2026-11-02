#!/usr/bin/env bash
set -eo pipefail
export ZKEEP_VISION_PYTHON="${ZKEEP_VISION_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm/bin/python}"
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
command="${1:-help}"
if (($#)); then shift; fi
case "$command" in
    driver|joy|joystick|moveit)
        source "$workspace_dir/tools/activate_ros.sh"
        if [[ "$command" == driver ]]; then
            exec ros2 launch zekeep_bringup driver.launch.py auto_enable:=false "$@"
        fi
        if [[ "$command" == joystick ]]; then
            exec ros2 launch zekeep_joystick cartesian_joystick.launch.py "$@"
        fi
        if [[ "$command" == moveit ]]; then
            exec ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep use_rviz:=false "$@"
        fi
        exec ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 "$@"
        ;;
esac
runtime="${LEROBOT_PYTHON:-${ACT_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/act/bin/python}}"
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
export PYTHONPATH="$workspace_dir/src/zekeep_act/src:$workspace_dir/src/zekeep_embodied/src:$workspace_dir/src/zekeep_teleop/src:${PYTHONPATH:-}"
if [[ "$command" == help ]]; then command=--help; fi
exec "$runtime" -m zekeep_act.cli "$command" "$@"
