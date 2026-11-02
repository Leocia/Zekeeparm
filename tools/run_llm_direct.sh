#!/usr/bin/env bash
# Dedicated DeepSeek-only HTTP CONNECT exit. Leaves existing Clash settings intact.
set -euo pipefail
llm_interface="${1:?Usage: bash tools/run_llm_direct.sh <physical-interface>}"
[[ "$llm_interface" =~ ^[a-zA-Z0-9_.:-]+$ ]] || { echo 'Invalid interface' >&2; exit 1; }
llm_proxy_bin="${ZKEEP_LLM_PROXY_BIN:-/usr/bin/verge-mihomo}"
[[ -x "$llm_proxy_bin" ]] || { echo 'Mihomo executable unavailable' >&2; exit 1; }
llm_runtime_dir="$(mktemp -d /tmp/zekeep-llm-direct.XXXXXX)"
cat > "$llm_runtime_dir/config.yaml" <<YAML
mixed-port: 17897
bind-address: 127.0.0.1
allow-lan: false
mode: rule
interface-name: $llm_interface
log-level: warning
dns:
  enable: true
  enhanced-mode: redir-host
  nameserver: [223.5.5.5, 119.29.29.29]
rules:
  - DOMAIN,api.deepseek.com,DIRECT
  - MATCH,REJECT
YAML
"$llm_proxy_bin" -d "$llm_runtime_dir" -f "$llm_runtime_dir/config.yaml" &
llm_proxy_pid=$!
trap 'kill "$llm_proxy_pid" 2>/dev/null || true' EXIT
export ZKEEP_LLM_PROXY_URL=http://127.0.0.1:17897
python3 -m zekeep_llm.web_agent
