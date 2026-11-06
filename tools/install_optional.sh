#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/tools/environment.sh"
module="${1:-help}"
case "$module" in
  help|-h|--help)
    printf '%s\n' '用法：bash tools/install_optional.sh collect|models|sim|web-sim' \
      '按需安装依赖；不会下载模型权重、连接设备或执行 sudo。' \
      'collect：数采环境（主从插件一并安装）；实际采集另需 ros 和 vision。' \
      'models：ACT/π0.5 共用模型环境；离线训练不要求 ros、vision 或 teleop。' \
      'sim：先运行 bash setup.sh ros，再安装 ROS MuJoCo。' \
      'web-sim：独立网页仿真；只需系统 Python venv 和 Node.js 22+/npm，不要求 ROS/Conda。' \
      '完整步骤与系统依赖：docs/INSTALL_PORTABLE.md；apps/ 独立交付，不随 GitHub 发布。'
    exit 0 ;;
  collect|models|sim|web-sim|studio) ;;
  *) printf '未知选装模块：%s\n' "$module" >&2; exit 2 ;;
esac
[[ $# -eq 1 ]] || { printf '每次只接受一个选装模块。\n' >&2; exit 2; }
cd -- "$workspace_dir"
if [[ "$module" == studio && ! -f apps/studio/environment.yml ]]; then
  printf 'Studio 未随 GitHub 仓库发布；请先另行准备 apps/studio 源码。\n' >&2
  exit 1
fi
if [[ "$module" == web-sim || "$module" == studio ]]; then
  for tool in node npm; do
    command -v "$tool" >/dev/null || { printf '先在当前终端安装 Node.js 和 npm：缺少 %s\n' "$tool" >&2; exit 1; }
  done
fi
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export PYTHONNOUSERSITE=1
forge="${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}"
conda="$forge/bin/conda"
create_conda() {
  local prefix="$1"
  [[ -x "$conda" ]] || { printf '缺少 Conda：%s\n' "$conda" >&2; exit 1; }
  if [[ ! -f "$prefix/conda-meta/history" ]]; then
    "$conda" create --override-channels -c conda-forge -p "$prefix" python=3.10 -y
  fi
}
case "$module" in
  collect)
    prefix="$forge/envs/embodied"
    create_conda "$prefix"
    "$conda" run -p "$prefix" python -m pip install --upgrade pip
    "$conda" run -p "$prefix" python -m pip install \
      -r src/zekeep_embodied/requirements-embodied.txt -e src/zekeep_teleop -e src/zekeep_embodied
    "$conda" run -p "$prefix" python -m pip check
    "$conda" run -p "$prefix" python -c \
      'from lerobot_teleoperator_zhongling import ZhonglingLeader, ZhonglingLeaderConfig; import zekeep_embodied.capture; print("数采与主臂插件导入：OK")'
    ;;
  models)
    prefix="$forge/envs/act"
    create_conda "$prefix"
    "$conda" run -p "$prefix" python -m pip install --upgrade pip
    "$conda" run -p "$prefix" python -m pip install \
      torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
    "$conda" run -p "$prefix" python -m pip install \
      -r src/zekeep_embodied/requirements-models.txt -e src/zekeep_embodied -e src/zekeep_act
    "$conda" run -p "$prefix" python -m pip check
    "$conda" run -p "$prefix" python -c \
      'import torch, torchvision; from torchcodec.decoders import VideoDecoder; from lerobot.policies.pi05.modeling_pi05 import PI05Policy; from zekeep_act import cli; print("ACT/pi0.5 imports: OK")'
    ;;
  sim)
    ros_venv="${ZKEEP_ROS_VENV:-$workspace_dir/.venv-ros}"
    [[ -x "$ros_venv/bin/python" ]] || { printf '先运行 bash setup.sh ros 创建 ROS 环境：%s\n' "$ros_venv" >&2; exit 1; }
    set +u
    source tools/activate_ros.sh
    set -u
    "$ros_venv/bin/python" -m pip install mujoco==3.10.0
    python -m colcon build --base-paths src --packages-select zekeep_sim --symlink-install
    ;;
  web-sim)
    node -e 'if (Number(process.versions.node.split(".")[0]) < 22) { console.error("需要 Node.js 22+"); process.exit(1); }'
    /usr/bin/python3 -m venv web_mujoco/.venv
    web_mujoco/.venv/bin/python -m pip install -r web_mujoco/requirements.txt
    export PATH="$workspace_dir/web_mujoco/.venv/bin:$PATH"
    (cd web_mujoco && npm ci --engine-strict && npm run build)
    ;;
  studio)
    [[ -x "$conda" ]] || { printf '缺少 Conda：%s\n' "$conda" >&2; exit 1; }
    prefix="$forge/envs/zekeep-studio"
    if [[ -f "$prefix/conda-meta/history" ]]; then
      "$conda" env update -p "$prefix" -f apps/studio/environment.yml
    else
      "$conda" env create -p "$prefix" -f apps/studio/environment.yml
    fi
    (cd apps/studio && npm ci --engine-strict)
    ;;
esac
printf '选装依赖已完成：%s；模型权重仍需按文档单独准备。\n' "$module"
