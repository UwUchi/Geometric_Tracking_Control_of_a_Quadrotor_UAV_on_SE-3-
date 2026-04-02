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

bag_path="${1:-}"
play_rate="${PLAY_RATE:-0.5}"
use_rviz="${USE_RVIZ:-true}"
show_error_markers="${SHOW_ERROR_MARKERS:-false}"
path_max_points="${PATH_MAX_POINTS:-2000}"
rviz_config="${RVIZ_CONFIG:-${workspace_dir}/install/quad_se3_py/share/quad_se3_py/rviz/quad_recording.rviz}"
launch_pid=""
play_pid=""

if [ -z "${bag_path}" ]; then
  latest_bag="$(find "${workspace_dir}/bags" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
  if [ -z "${latest_bag}" ]; then
    echo "No bag found under ${workspace_dir}/bags" >&2
    exit 1
  fi
  bag_path="${latest_bag}"
fi

cleanup() {
  if [ -n "${play_pid}" ] && kill -0 "${play_pid}" 2>/dev/null; then
    kill -INT "${play_pid}" 2>/dev/null || true
    wait "${play_pid}" 2>/dev/null || true
  fi
  if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

echo "Starting playback visualization for bag: ${bag_path}"
ROS_LOG_DIR="${workspace_dir}/log/ros2" \
  ros2 launch quad_se3_py playback_viz.launch.py \
  use_rviz:="${use_rviz}" \
  use_sim_time:=true \
  show_error_markers:="${show_error_markers}" \
  path_max_points:="${path_max_points}" \
  rviz_config:="${rviz_config}" &
launch_pid=$!

sleep 1

echo "Playing bag at ${play_rate}x with /clock enabled..."
ros2 bag play "${bag_path}" --clock 50.0 --rate "${play_rate}" &
play_pid=$!

wait "${play_pid}"
play_pid=""

if [ -n "${launch_pid}" ] && kill -0 "${launch_pid}" 2>/dev/null; then
  kill -INT "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
fi

launch_pid=""
trap - EXIT INT TERM
