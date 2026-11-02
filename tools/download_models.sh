#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
die() { printf '错误：%s\n' "$*" >&2; exit 1; }
download_checked() {
  local url="$1" destination="$2" expected="$3" temporary
  mkdir -p "$(dirname -- "$destination")"
  if [[ -f "$destination" ]] && printf '%s  %s\n' "$expected" "$destination" | sha256sum -c - >/dev/null 2>&1; then
    printf '已存在：%s\n' "$destination"
    return
  fi
  temporary="$(mktemp "${destination}.part.XXXXXX")"
  if ! curl -fL --retry 3 --retry-delay 2 -o "$temporary" "$url"; then
    rm -f -- "$temporary"
    return 1
  fi
  if ! printf '%s  %s\n' "$expected" "$temporary" | sha256sum -c -; then
    rm -f -- "$temporary"
    die "下载文件校验失败：$destination"
  fi
  mv -f -- "$temporary" "$destination"
}

case "${1:-help}" in
  base)
download_checked \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26l-seg.pt \
  "$workspace_dir/src/zekeep_grasp/models/yoloe-26l-seg.pt" \
  a612d2d505f24e14d87ec82d688b823b6cb600646664f16125ce6c84ce360da9
download_checked \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts \
  "$workspace_dir/mobileclip2_b.ts" \
  35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f
    # Ultralytics resolves text weights from cwd; support both documented launch directories.
    ln -sfn ../../mobileclip2_b.ts "$workspace_dir/src/zekeep_grasp/mobileclip2_b.ts"
    ;;
  graspnet)
    [[ "${ACCEPT_GRASPNET_LICENSE:-}" == YES ]] || die "先阅读 docs/THIRD_PARTY_NOTICES.md；接受许可后设置 ACCEPT_GRASPNET_LICENSE=YES"
    download_checked \
      'https://drive.usercontent.google.com/download?id=1hd0G8LN6tRpi4742XOTEisbTXNZ-1jmk&export=download&confirm=t' \
      "$workspace_dir/third_party/graspnet-baseline/checkpoints/checkpoint-rs.tar" \
      60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868
    ;;
  help|-h|--help) printf '%s\n' '用法：bash tools/download_models.sh base|graspnet' '只下载模型，不安装依赖、不连接设备；已有正确文件不会重复下载。' ;;
  *) die "未知模型组：$1" ;;
esac
