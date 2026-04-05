#!/usr/bin/env bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  # Load the base ROS environment first.
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ ! -f "${workspace_dir}/install/setup.bash" ]; then
  echo "Workspace setup not found: ${workspace_dir}/install/setup.bash" >&2
  echo "Run 'colcon build' in ${workspace_dir} first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${workspace_dir}/install/setup.bash"

set -euo pipefail

case_name="${1:-case1_helix}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${2:-${workspace_dir}/bags/${case_name}_${timestamp}}"
export ROS_LOG_DIR="${workspace_dir}/log/ros2"

mkdir -p "${workspace_dir}/bags"
mkdir -p "${ROS_LOG_DIR}"

exec ros2 bag record \
  --output "${output_dir}" \
  /quad_state \
  /trajectory_epoch \
  /trajectory \
  /control_input
