#!/usr/bin/env bash
# Record a motion failure without sending any robot commands.
set -eo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "用法: bash tools/record_motion_debug.sh [保存目录]"
  echo "先启动机械臂和 MoveIt，再录包，最后执行一次搬运。"
  echo "失败后等 3 秒，按 Ctrl+C 停止；提供整个保存目录。"
  exit 0
fi
if (( $# > 1 )); then
  echo "只接受一个保存目录参数。" >&2
  exit 2
fi

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u

output=${1:-"$workspace/data/motion_debug_$(date +%Y%m%d_%H%M%S_%N)"}
mkdir -p -- "$(dirname -- "$output")"
# Refuse existing directories so previous recordings cannot be overwritten.
mkdir -- "$output"
output=$(cd -- "$output" && pwd)

cat > "$output/context.txt" <<EOF
Recording started: $(date --iso-8601=seconds)
Workspace: $workspace
ROS_DISTRO: ${ROS_DISTRO:-}
ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}
RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}
Action feedback error uses desired - actual.
The controller may stop feedback during final settling; use joint_states
with the last desired target and the planned trajectory during that interval.
EOF
cp -- "$workspace/src/zekeepcontroller/zekeepcontroller/ros_actions.py" "$output/ros_actions_source.py"
cp -- "$workspace/src/zekeep_bringup/config/zekeep_hardware.yaml" "$output/hardware_source.yaml"

echo "保存到: $output"
echo "等录包出现 Subscribed to topic 后，再执行一次搬运。"
echo "失败后等 3 秒，按 Ctrl+C；请提供整个目录（包含 bag）。"

# Discovery stays enabled, including feedback topics that appear later.
# Humble records action feedback/status topics, not action goal/result services.
exec ros2 bag record --storage sqlite3 --include-hidden-topics \
  --output "$output/bag" \
  --regex '^(/zekeep/.*|/joint_states|/rosout|/tf|/tf_static|/clock|(.*/)?display_planned_path|/execute_trajectory/_action/.*|/move_action/_action/.*)$'
