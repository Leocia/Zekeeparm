#!/usr/bin/env bash
set -e
zekeep_launcher_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$zekeep_launcher_dir/environment.sh"
exec /usr/bin/python3 "$zekeep_launcher_dir/web_grasp_launcher.py" "$@"
