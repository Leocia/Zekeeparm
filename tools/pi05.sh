#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
command="${1:-help}"
case "$command" in
    export|check|stats|train|predict|serve|shadow|execute|rviz|replay|joy)
        exec bash "$workspace_dir/tools/pi05-model.sh" "$@"
        ;;
    *) exec bash "$workspace_dir/tools/collect.sh" "$@" ;;
esac
