#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
package_dir="$workspace_dir/src/zekeep_embodied"
config="$package_dir/configs/record.yaml"
command="${1:-help}"
if (($#)); then shift; fi

usage() {
    cat <<'HELP'
用法：./tools/pi05.sh <子命令> [参数]

  doctor                         检查环境、基础权重和 GPU；不连接设备
  driver [ROS launch 参数]       启动真机驱动，默认不使能电机
  joystick [ROS launch 参数]     启动关节手柄控制；电机须现场手动使能
  joy [ROS 参数]                仅发布 /joy，供策略执行读取 deadman
  camera-check [--seconds 10]    仅检查双相机
  record --output RAW_DIR        只读录制已有 ROS 手柄控制
  record-leader --output RAW_DIR  主臂遥操并录制示范；会连接并使能从臂
  check-raw EPISODE_DIR          检查一条原始示范
  preview-raw EPISODE_DIR --output VIDEO.mp4
  export RAW_DIR --repo-id local/新数据集 [--fps 9]
  check --repo-id local/新数据集
  stats --repo-id local/新数据集
  train --repo-id local/新数据集 --exp-name 新实验 --num-train-steps 1000
  predict --repo-id local/新数据集 --exp-name 新实验 [--frame-index 0] [--output FILE.json]
  serve --repo-id local/新数据集 --exp-name 新实验
  shadow --repo-id local/新数据集 --exp-name 新实验 [--save-preview FILE.json]
  execute --repo-id local/新数据集 --exp-name 新实验
  replay FILE.json              回放 shadow 保存的预测；不连接实机
  rviz                          打开预测回放专用 RViz

采集、相机和运行命令可加 --config FILE.yaml。每个步骤独立启动，不自动串行运动。
execute 保留交互 MOVE 确认和按住 deadman 才运动的保护，不自动使能电机。
record-leader 直接占用主从臂串口，勿同时启动 ROS 驱动；退出会释放从臂力矩。
离线 predict 文件用于检查动作数值；replay 只接受 shadow --save-preview 文件。
详细步骤：src/zekeep_embodied/docs/pi05-entry.md
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
        status=0
        for path in \
            "${EMBODIED_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/embodied/bin/python}" \
            "${ZKEEP_VISION_PYTHON:-${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}/envs/rebotarm/bin/python}" \
            "$workspace_dir/third_party/openpi/.venv/bin/python" \
            "$workspace_dir/.venv-ros/bin/python" \
            /opt/ros/humble/setup.bash "$workspace_dir/install/setup.bash" \
            "$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_base_pytorch/model.safetensors"; do
            if [[ -f "$path" ]]; then
                printf 'OK %s\n' "$path"
            else
                printf '缺少 %s\n' "$path" >&2
                status=1
            fi
        done
        if ! nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader; then
            status=1
        fi
        printf '模型命令必须显式选择数据集和实验；doctor 不验证数据或模型效果。\n'
        exit "$status"
        ;;
    driver|joystick|joy|rviz)
        case "$command" in
            driver) exec bash "$workspace_dir/tools/env.sh" ros ros2 launch zekeep_bringup driver.launch.py auto_enable:=false "$@" ;;
            joystick) exec bash "$workspace_dir/tools/env.sh" ros ros2 launch zekeep_joystick joystick.launch.py "$@" ;;
            joy) exec bash "$workspace_dir/tools/env.sh" ros ros2 run joy joy_node --ros-args -p deadzone:=0.05 -p autorepeat_rate:=20.0 "$@" ;;
            rviz) exec bash "$workspace_dir/tools/env.sh" ros ros2 launch "$package_dir/tools/pi05_preview.launch.py" "$@" ;;
        esac
        ;;
    camera-check)
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.record \
            --camera-check --config "$config" --seconds 10 "$@"
        ;;
    record)
        require_option --output "$@"
        exec bash "$package_dir/tools/run_ros.sh" -m zekeep_embodied.record \
            --joystick --config "$config" "$@"
        ;;
    record-leader)
        require_option --output "$@"
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.record \
            --execute --config "$config" "$@"
        ;;
    check-raw|preview-raw)
        action=check
        if [[ "$command" == preview-raw ]]; then action=preview; fi
        exec bash "$package_dir/tools/run.sh" -m zekeep_embodied.dataset "$action" "$@"
        ;;
    export|check|stats|train|predict|serve|shadow|execute)
        require_option --repo-id "$@"
        case "$command" in
            train|predict|serve|shadow|execute) require_option --exp-name "$@" ;;
        esac
        case "$command" in
            train)
                require_option --num-train-steps "$@"
                exec bash "$package_dir/tools/run_pi05.sh" -m zekeep_embodied.pi05 train-qlora "$@"
                ;;
            serve)
                exec bash "$package_dir/tools/run_pi05.sh" -m zekeep_embodied.pi05_policy_server "$@"
                ;;
            shadow|execute)
                exec bash "$package_dir/tools/run_ros.sh" -m zekeep_embodied.pi05_runtime \
                    --config "$config" "--$command" "$@"
                ;;
            *) exec bash "$package_dir/tools/run_pi05.sh" -m zekeep_embodied.pi05 "$command" "$@" ;;
        esac
        ;;
    replay)
        exec bash "$package_dir/tools/run_ros.sh" -m zekeep_embodied.pi05_preview "$@"
        ;;
    *) printf '未知子命令：%s\n' "$command" >&2; usage >&2; exit 2 ;;
esac
