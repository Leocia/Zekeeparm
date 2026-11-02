#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
package_dir="$workspace_dir/src/zekeep_embodied"
config="$package_dir/configs/record.yaml"
command="${1:-help}"
if (($#)); then shift; fi

usage() {
    cat <<'HELP'
用法：./tools/pi05-model.sh <子命令> [参数]

  doctor                         检查共用 LeRobot 环境、基础权重是否存在及 CUDA；不加载模型
  export RAW_DIR --repo-id local/新数据集 [--fps 10]  导出共用 v3
  check --repo-id local/新数据集
  stats --repo-id local/新数据集
  train --repo-id local/新数据集 --exp-name 新实验 --num-train-steps 1000
                                 VLM LoRA + Action Expert 全量；不量化基础权重
  predict --repo-id local/新数据集 --exp-name 新实验 [--frame-index 0] [--output FILE.json]
  serve --repo-id local/新数据集 --exp-name 新实验
  shadow --repo-id local/新数据集 --exp-name 新实验 [--save-preview FILE.json]
  execute --repo-id local/新数据集 --exp-name 新实验
  joy [ROS 参数]                 手柄调试兼容入口；模型执行无需启动
  replay FILE.json               回放 shadow 保存的预测；不连接实机
  rviz                           打开预测回放专用 RViz

export、check、stats、train 在云端模型环境运行；serve、shadow、execute 用于本地验证。
execute 输入 MOVE 确认后全局按住 m 运动、松开暂停；需本机 X11 桌面，不自动使能。
driver 使用 ./tools/collect.sh driver。
check/stats/train/predict/serve 可加 --dataset-root 指向已有 ACT 的 v3 train 目录，无须再次导出。
当前训练仅在结束时保存 LoRA、完整动作专家和动作头，不支持断点续训。
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
        exec bash "$package_dir/tools/run_pi05.sh" -m zekeep_embodied.pi05 doctor "$@"
        ;;
    joy|rviz)
        set +u
        source "$workspace_dir/tools/activate_ros.sh"
        set -u
        case "$command" in
            joy) exec ros2 run joy joy_node --ros-args -p deadzone:=0.05 -p autorepeat_rate:=20.0 "$@" ;;
            rviz) exec ros2 launch "$package_dir/tools/pi05_preview.launch.py" "$@" ;;
        esac
        ;;
    export|check|stats|train|predict|serve|shadow|execute)
        require_option --repo-id "$@"
        case "$command" in
            train|predict|serve|shadow|execute) require_option --exp-name "$@" ;;
        esac
        case "$command" in
            train)
                require_option --num-train-steps "$@"
                exec bash "$package_dir/tools/run_pi05.sh" -m zekeep_embodied.pi05 train-lora "$@"
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
