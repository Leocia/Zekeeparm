#!/usr/bin/env bash
# Fetch optional sources without installing dependencies or accessing devices.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ACCEPT_GRASPNET_LICENSE:-}" != YES ]]; then
  printf '%s\n' '请先阅读 docs/THIRD_PARTY_NOTICES.md 和 GraspNet 上游许可；接受后设置 ACCEPT_GRASPNET_LICENSE=YES。' >&2
  exit 2
fi
graspnet_dir="$workspace_dir/third_party/graspnet-baseline"
mkdir -p "$workspace_dir/third_party"
if [[ ! -d "$graspnet_dir" ]]; then
  git clone --depth 1 https://github.com/graspnet/graspnet-baseline.git "$graspnet_dir"
fi
if [[ ! -d "$graspnet_dir/graspnetAPI" ]]; then
  git clone --depth 1 https://github.com/graspnet/graspnetAPI.git "$graspnet_dir/graspnetAPI"
fi
for source_file in models/graspnet.py pointnet2/setup.py knn/setup.py graspnetAPI/setup.py; do
  if [[ ! -f "$graspnet_dir/$source_file" ]]; then
    printf 'GraspNet 源码不完整，缺少：%s；请核对下载目录，脚本不会覆盖已有文件。\n' "$graspnet_dir/$source_file" >&2
    exit 1
  fi
done
printf 'GraspNet 源码已就绪：%s；Python 依赖、CUDA 扩展和权重仍需安装。\n' "$graspnet_dir"
