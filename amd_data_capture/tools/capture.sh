#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
command="${1:-help}"
if (($#)); then shift; fi
config="$root/src/zekeep_embodied/configs/record.yaml"
runtime="$root/.venv-capture/bin/python"
export PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
unset PYTHONPATH
case "$command" in
  help|-h|--help)
    cat <<'HELP'
用法：./tools/capture.sh <命令> [参数]
  doctor                         依赖与配置检查，不连接设备
  self-check                     临时目录内录写、校验和视频回看测试，不连接设备
  camera-check [--seconds 10]     双相机检查，不连接机械臂
  record-leader --output DIR      主从臂数采；启动会使能从臂并立即跟随
  driver [ROS launch 参数]        手柄数采驱动，默认不使能
  moveit [ROS launch 参数]        手柄数采的 TF 与 MoveIt
  joystick [ROS launch 参数]      手柄控制，需要已运行的驱动和 MoveIt
  record --output DIR             只读录制现有 ROS 手柄控制
  check-raw EPISODE_DIR           检查原始数据
  preview-raw EPISODE_DIR --output VIDEO.mp4
  audit RAW_DIR --output REPORT.json
doctor、camera-check、record、record-leader 可加 --config FILE.yaml。
录制：s 开始、p 成功保存、w 失败保存、d 放弃、e 退出；每次按回车。
主从模式退出会释放从臂力矩，先扶稳机械臂；不能与 ROS 驱动同时占用从臂。
HELP
    exit ;;
  driver|moveit|joystick|record)
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
    source "$root/tools/activate_ros.sh"
    ;;
esac
set -u
case "$command" in
  doctor) exec "$runtime" "$root/tools/doctor.py" --config "$config" "$@" ;;
  self-check) exec "$runtime" -m unittest discover -s "$root/tests" -v "$@" ;;
  camera-check)
    exec "$runtime" -m zekeep_embodied.record --camera-check --config "$config" --seconds 10 "$@" ;;
  record|record-leader)
    output=false
    for arg in "$@"; do
      case "$arg" in --output|--output=*|-h|--help) output=true ;; esac
    done
    if [[ "$output" != true ]]; then echo '请用 --output 明确指定数据目录。' >&2; exit 2; fi
    mode=--execute
    if [[ "$command" == record ]]; then mode=--joystick; fi
    exec "$runtime" -m zekeep_embodied.record "$mode" --config "$config" "$@" ;;
  driver) exec ros2 launch zekeep_bringup driver.launch.py auto_enable:=false "$@" ;;
  moveit) exec ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep use_rviz:=false "$@" ;;
  joystick) exec ros2 launch zekeep_joystick cartesian_joystick.launch.py "$@" ;;
  check-raw) exec "$runtime" -m zekeep_embodied.dataset check "$@" ;;
  preview-raw) exec "$runtime" -m zekeep_embodied.dataset preview "$@" ;;
  audit) exec "$runtime" "$root/tools/audit.py" "$@" ;;
  *) echo "未知命令：$command；使用 help 查看数采命令。" >&2; exit 2 ;;
esac
