#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec "${ZKEEP_STUDIO_PYTHON:-python3}" studio.py "$@"
