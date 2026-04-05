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
show_error_markers="${SHOW_ERROR_MARKERS:-false}"
path_max_points="${PATH_MAX_POINTS:-2000}"
rviz_config="${RVIZ_CONFIG:-$(cd "${workspace_dir}" && pwd)/install/quad_se3_py/share/quad_se3_py/rviz/quad_recording.rviz}"
case_name="case1_helix"
reference_time_offset_sec="${REFERENCE_TIME_OFFSET_SEC:-0.0}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${workspace_dir}/bags/${case_name}_${timestamp}"
metadata_path="${output_dir}/experiment_metadata.json"
launch_pid=""
record_pid=""

update_metadata_after_recording() {
  local detected_start_time
  detected_start_time="$(
    python3 - "${output_dir}" <<'PY'
import json
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_dir = Path(sys.argv[1])
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', ''),
)
topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
epoch_type = get_message(topic_types['/trajectory_epoch'])
quad_state_type = get_message(topic_types['/quad_state'])

start_time_sec = None
epoch_source = 'first_quad_state_stamp'
while reader.has_next():
    topic, data, timestamp_ns = reader.read_next()
    if topic == '/trajectory_epoch':
        message = deserialize_message(data, epoch_type)
        start_time_sec = float(message.sec) + float(message.nanosec) * 1e-9
        epoch_source = 'trajectory_epoch_topic'
        break
    if topic != '/quad_state':
        continue
    message = deserialize_message(data, quad_state_type)
    start_time_sec = float(message.stamp.sec) + float(message.stamp.nanosec) * 1e-9
    break

if start_time_sec is None:
    raise RuntimeError('No /quad_state sample found while updating bag metadata.')

print(json.dumps({
    'trajectory_start_time_sec': start_time_sec,
    'trajectory_epoch_source': epoch_source,
}))
PY
  )"
  local epoch_source
  epoch_source="$(
    python3 - "${detected_start_time}" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
print(payload['trajectory_epoch_source'])
PY
  )"
  detected_start_time="$(
    python3 - "${detected_start_time}" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
print(f"{payload['trajectory_start_time_sec']:.9f}")
PY
  )"

  printf '%s\n' \
    '{' \
    "  \"case_name\": \"${case_name}\"," \
    "  \"trajectory_mode\": \"${trajectory_mode}\"," \
    "  \"trajectory_start_time_sec\": ${detected_start_time}," \
    "  \"reference_time_offset_sec\": ${reference_time_offset_sec}," \
    "  \"trajectory_epoch_source\": \"${epoch_source}\"," \
    '  "reference_source": "time_function"' \
    '}' > "${metadata_path}"
}

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

echo "Starting bag recording..."
"${script_dir}/record_bag.sh" "${case_name}" "${output_dir}" &
record_pid=$!

sleep 1

mkdir -p "${output_dir}"
printf '%s\n' \
  '{' \
  "  \"case_name\": \"${case_name}\"," \
  "  \"trajectory_mode\": \"${trajectory_mode}\"," \
  '  "trajectory_start_time_sec": null,' \
  "  \"reference_time_offset_sec\": ${reference_time_offset_sec}," \
  '  "trajectory_epoch_source": "pending_first_quad_state_stamp",' \
  '  "reference_source": "time_function"' \
  '}' > "${metadata_path}"

echo "Starting simulation with trajectory_mode=${trajectory_mode}..."
ROS_LOG_DIR="${workspace_dir}/log/ros2" \
  ros2 launch quad_se3_py sim_viz.launch.py \
  use_rviz:="${use_rviz}" \
  trajectory_mode:="${trajectory_mode}" \
  trajectory_start_time_sec:=0.0 \
  reference_time_offset_sec:="${reference_time_offset_sec}" \
  show_error_markers:="${show_error_markers}" \
  path_max_points:="${path_max_points}" \
  rviz_config:="${rviz_config}" &
launch_pid=$!

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
update_metadata_after_recording

echo "Analyzing latest bag..."
python3 "${script_dir}/analyze_bag.py" "${output_dir}"
