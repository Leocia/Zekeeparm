#!/usr/bin/env bash
# Run commands in one project environment without changing the parent shell.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  printf '请执行 bash tools/env.sh <环境> [命令]，不要 source。\n' >&2
  return 2
fi
set -eo pipefail

profile="${1:-help}"
case "$profile" in
  help|-h|--help)
    printf '%s\n' '用法：bash tools/env.sh <base|ros|vision|teleop|vla|model|act|studio> [命令及参数]'
    printf '%s\n' '省略命令时进入子终端，exit 返回；base 可在环境尚未安装时使用。'
    exit 0 ;;
  base|ros|vision|teleop|vla|model|act|studio) shift ;;
  *) printf '未知环境：%s\n' "$profile" >&2; exit 2 ;;
esac

export ZKEEP_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ZKEEP_MINIFORGE_DIR="${ZKEEP_MINIFORGE_DIR:-$HOME/miniforge3}"
export EMBODIED_PYTHON="${EMBODIED_PYTHON:-$ZKEEP_MINIFORGE_DIR/envs/embodied/bin/python}"
export ZKEEP_VISION_PYTHON="${ZKEEP_VISION_PYTHON:-$ZKEEP_MINIFORGE_DIR/envs/rebotarm/bin/python}"
export PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
if [[ -n "${CUDA_HOME:-}" ]]; then export PATH="$CUDA_HOME/bin:$PATH"; fi
cd -- "$ZKEEP_WS"
prefix=""
if (($# == 0)); then set -- /bin/bash --noprofile --norc -i; fi

case "$profile" in
  base) ;;
  ros) source "$ZKEEP_WS/tools/activate_ros.sh" ;;
  *)
    case "$profile" in
      vision) prefix="$(dirname -- "$(dirname -- "$ZKEEP_VISION_PYTHON")")" ;;
      model|act) prefix="$(dirname -- "$(dirname -- "${LEROBOT_PYTHON:-${ACT_PYTHON:-$ZKEEP_MINIFORGE_DIR/envs/act/bin/python}}")")" ;;
      teleop) prefix="$ZKEEP_MINIFORGE_DIR/envs/lerobot" ;;
      vla)
        prefix="$(dirname -- "$(dirname -- "$EMBODIED_PYTHON")")"
        export JAX_PLATFORMS="${JAX_PLATFORMS:-cpu}"
        ;;
      studio) prefix="$ZKEEP_MINIFORGE_DIR/envs/zekeep-studio" ;;
    esac
    if [[ ! -x "$ZKEEP_MINIFORGE_DIR/bin/conda" ]]; then
      printf '缺少 Miniforge；按 README 安装或设置 ZKEEP_MINIFORGE_DIR。\n' >&2
      exit 1
    fi
    ;;
esac
if [[ -n "${prefix:-}" && ! -x "$prefix/bin/python" ]]; then
  printf '环境未安装：%s；请按 README 对应章节准备。\n' "$prefix" >&2
  exit 1
fi
case "$profile" in
  base|ros) exec "$@" ;;
  *) exec "$ZKEEP_MINIFORGE_DIR/bin/conda" run --no-capture-output -p "$prefix" "$@" ;;
esac
