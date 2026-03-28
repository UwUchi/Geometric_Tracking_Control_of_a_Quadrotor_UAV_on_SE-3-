#!/usr/bin/env bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
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

trajectory_mode="${1:-paper_case_1_helix}"
use_rviz="${USE_RVIZ:-true}"
case_name="case1_helix"
launch_pid=""
record_pid=""

cleanup() {
  if [ -n "${record_pid}" ] && kill -0 "${record_pid}" 2>/dev/null; then
    kill -INT "${record_pid}" 2>/dev/null || true
    wait "${record_pid}" 2>/dev/null || true
  fi
  if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

echo "Starting simulation with trajectory_mode=${trajectory_mode}..."
ROS_LOG_DIR="${workspace_dir}/log/ros2" \
  ros2 launch quad_se3_py sim_viz.launch.py \
  use_rviz:="${use_rviz}" \
  trajectory_mode:="${trajectory_mode}" &
launch_pid=$!

sleep 3

echo "Starting bag recording..."
"${script_dir}/record_bag.sh" "${case_name}" &
record_pid=$!

echo "Simulation and recording are running."
echo "Press Ctrl+C when you want to stop and analyze the latest bag."

wait "${launch_pid}" || true

if [ -n "${record_pid}" ] && kill -0 "${record_pid}" 2>/dev/null; then
  kill -INT "${record_pid}" 2>/dev/null || true
  wait "${record_pid}" 2>/dev/null || true
fi

record_pid=""
launch_pid=""
trap - EXIT INT TERM

echo "Analyzing latest bag..."
python3 "${script_dir}/analyze_bag.py" --case-name "${case_name}"
