#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
package_dir="$workspace_dir/src/zekeep_embodied"
config="$package_dir/configs/record.yaml"
command="${1:-help}"
if (($#)); then shift; fi

usage() {
    cat <<'HELP'
用法：./tools/collect.sh <子命令> [参数]

  doctor                         检查数据环境和配置路径；不连接设备，不要求模型/GPU
  driver [ROS launch 参数]        启动真机驱动，默认不使能电机
  joystick [ROS launch 参数]      启动末端手柄控制；需 MoveIt，电机须现场手动使能
  camera-check [--seconds 10]     检查双相机
  record --output RAW_DIR [--episodes 50]  只读录制已有 ROS 手柄控制
  record-leader --output RAW_DIR [--episodes 50]  主臂经 ROS 示教；不自动使能
  moveit / joystick / joy         启动已有 TF/规划、手柄控制或手柄消息节点
  check-raw EPISODE_DIR           检查一条原始示范
  preview-raw EPISODE_DIR --output VIDEO.mp4
  audit RAW_DIR --output REPORT.json
                                 批量质检，报告可用/排除示范；不修改原始数据
  bundle --train TRAIN_DIR --validation VAL_DIR --output DATA.tar
                                 检查人工划分，打包真实文件和 SHA-256 清单
  verify DATA.tar                校验数据包，不解压、不连接设备

ACT 与 π0.5 共用此入口和原始目录；后处理分别使用 act.sh prepare 与 pi05-model.sh export。
采集、相机和 doctor 可加 --config FILE.yaml（默认 record.yaml）。
record 只监听既有手柄控制；record-leader 需要唯一 ROS 驱动、现场手动使能、输入 f 对齐后跟随。
主臂采集保存后停止跟随；退出仍由驱动持力，须按真机规程支撑后失能。
两种示教方式使用不同原始目录。详细步骤：src/zekeep_embodied/README.md
HELP
}

require_option() {
    local option="$1" argument
    shift
    for argument in "$@"; do
        case "$argument" in
            -h|--help) return ;;
            "$option"|"$option"=*) return ;;
        esac
    done
    printf '缺少 %s；请显式选择新数据集、实验或训练步数。\n' "$option" >&2
    exit 2
}

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export JAX_PLATFORMS="${JAX_PLATFORMS:-cpu}"
export HF_DATASETS_CACHE="${HF_DATASETS_CACHE:-$workspace_dir/data/openpi/cache/huggingface_datasets}"

case "$command" in
    help|-h|--help) usage ;;
    doctor)
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.bundle doctor --config "$config" "$@"
        ;;
    audit|bundle|verify)
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.bundle "$command" "$@"
        ;;
    driver|joystick|moveit|joy)
        set +u
        source "$workspace_dir/tools/activate_ros.sh"
        set -u
        case "$command" in
            driver) exec ros2 launch zekeep_bringup driver.launch.py auto_enable:=false "$@" ;;
            joystick) exec ros2 launch zekeep_joystick cartesian_joystick.launch.py "$@" ;;
            moveit) exec ros2 launch zekeep_moveit_config hardware.launch.py arm_namespace:=zekeep use_rviz:=false "$@" ;;
            joy) exec ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 "$@" ;;
        esac
        ;;
    camera-check)
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.record \
            --camera-check --config "$config" --seconds 10 "$@"
        ;;
    record|record-leader)
        require_option --output "$@"
        exec bash "$package_dir/tools/run_ros.sh" -m zekeep_embodied.capture \
            "$command" --config "$config" "$@"
        ;;
    check-raw|preview-raw)
        action=check
        if [[ "$command" == preview-raw ]]; then action=preview; fi
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.dataset "$action" "$@"
        ;;
    *) printf '未知子命令：%s\n' "$command" >&2; usage >&2; exit 2 ;;
esac
