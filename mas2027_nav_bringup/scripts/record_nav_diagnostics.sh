#!/usr/bin/env bash
# Record one field navigation run for offline projection/terrain diagnosis.
#
# Usage:
#   ./src/mas2027_nav_bringup/scripts/record_nav_diagnostics.sh [RUN_DIR]
#
# Then start the normal navigation launch in another terminal. Ctrl-C here
# stops the bag and copies the executor/planner logs and performance CSVs into
# RUN_DIR. Set FIELD_RECORD_CLOUD=0 to omit /cloud_registered_full (about 1--
# 3 GB per five minutes); set FIELD_RUN_ROOT to change the default parent dir.
# For a bounded dry run, set FIELD_RECORD_SECONDS=30; otherwise stop with Ctrl-C.
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,11p' "$0"
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
if [[ -d "${SCRIPT_DIR}/../../../src/docs" ]]; then
  WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
else
  WORKSPACE="${MAS_NAV_WORKSPACE:-${PWD}}"
fi

RUN_ROOT="${FIELD_RUN_ROOT:-${WORKSPACE}/.scratch/field_runs}"
RUN_ID="${FIELD_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RUN_DIR="${1:-${RUN_ROOT}/${RUN_ID}}"
RECORD_CLOUD="${FIELD_RECORD_CLOUD:-1}"

if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # The user's overlay (if any) must still be sourced in this shell.
    source /opt/ros/jazzy/setup.bash
    set -u
  else
    echo "ros2 is not on PATH; source the ROS 2 environment first." >&2
    exit 2
  fi
fi

QOS="${SCRIPT_DIR}/../config/qos_diagnostics.yaml"
if [[ ! -f "${QOS}" ]]; then
  QOS="$(ros2 pkg prefix mas2027_nav_bringup 2>/dev/null || true)/share/mas2027_nav_bringup/config/qos_diagnostics.yaml"
fi
if [[ ! -f "${QOS}" ]]; then
  echo "QoS override file not found; expected config/qos_diagnostics.yaml." >&2
  exit 2
fi

mkdir -p "${RUN_DIR}/logs" "${RUN_DIR}/csv"
if [[ -e "${RUN_DIR}/bag" ]]; then
  echo "Refusing to overwrite existing bag: ${RUN_DIR}/bag" >&2
  exit 2
fi
touch "${RUN_DIR}/start.marker"

{
  echo "run_id=${RUN_ID}"
  echo "started=$(date -Is)"
  echo "hostname=$(hostname)"
  echo "workspace=${WORKSPACE}"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "rmw=${RMW_IMPLEMENTATION:-default}"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-0}"
  echo "record_cloud=${RECORD_CLOUD}"
  echo "run_dir=${RUN_DIR}"
  echo "qos_profile=${QOS}"
  uname -a
  if git -C "${WORKSPACE}/src" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "git_head=$(git -C "${WORKSPACE}/src" rev-parse HEAD)"
    echo "git_status:"
    git -C "${WORKSPACE}/src" status --short
  fi
} > "${RUN_DIR}/manifest.txt" 2>&1

# Snapshot the active configuration and robot geometry with the bag. This is
# more reliable than ros2 param dump after the nodes have exited.
mkdir -p "${RUN_DIR}/config"
for file in \
  "${WORKSPACE}"/src/mas2027_nav_executor/config/*.yaml \
  "${WORKSPACE}"/src/mas2027_nav_bringup/config/*.yaml \
  "${WORKSPACE}"/src/mas2027_robot_description/urdf/*.urdf; do
  [[ -f "${file}" ]] || continue
  cp -f "${file}" "${RUN_DIR}/config/$(basename "${file}")" 2>/dev/null || true
done

TOPICS=(
  /rosout
  /parameter_events
  /tf
  /tf_static
  /Odometry
  /goal_pose
  /cmd_vel
  /nav_executor/chassis_cmd
  /nav_executor/region_status
  /opt_path
  /nav_executor/minco_path
  /nav_executor/global_plan
  /nav_executor/debug/minco_trajectory
  /nav_executor/debug/global_plan
  /nav_executor/debug/safe_corridor
  /nav_executor/online_global_map
  /nav_executor/online_terrain_map
  /nav_executor/online_terrain_uphill
  /rog_map/layer_type
  /rog_map/layer_value
  /rog_map/terrain_label
  /rog_map/layer_height_delta
  /rog_map/field
  /rog_map/occupied
  /rog_map/raw_occupied
)
if [[ "${RECORD_CLOUD}" != "0" ]]; then
  TOPICS+=(/cloud_registered_full)
fi

BAG_ARGS=(--disable-keyboard-controls -s mcap -o "${RUN_DIR}/bag" --qos-profile-overrides-path "${QOS}")
# Prefer the fast zstd preset when this rosbag2 build supports it.
if ros2 bag record --help 2>&1 | grep -q -- '--storage-preset-profile'; then
  BAG_ARGS+=(--storage-preset-profile zstd_fast)
fi

echo "Recording to ${RUN_DIR}/bag"
echo "Topics: ${TOPICS[*]}"
echo "Now start the normal navigation launch in another terminal."
echo "Press Ctrl-C here to stop recording and collect logs/CSVs."

# Capture the running ROS graph while the nodes still exist. A post-run param
# dump only records "Node not found" when navigation stops before the bag.
capture_runtime_graph() {
  local pending="${RUN_DIR}/.graph_capture_pending"
  while [[ ! -e "${RUN_DIR}/recording_stopped.marker" ]]; do
    if timeout 3 ros2 node list > "${pending}.nodes" 2>&1 &&
      grep -Fxq /nav_executor "${pending}.nodes" &&
      grep -Fxq /nav_executor_planner "${pending}.nodes"; then
      mv -f "${pending}.nodes" "${RUN_DIR}/nodes.txt"
      if timeout 3 ros2 topic list -t > "${pending}.topics" 2>&1; then
        mv -f "${pending}.topics" "${RUN_DIR}/topics.txt"
      fi
      if [[ ! -f "${RUN_DIR}/params_planner.yaml" ]] &&
        timeout 3 ros2 param dump /nav_executor_planner > "${pending}.planner" 2>&1 &&
        grep -q 'ros__parameters:' "${pending}.planner"; then
        mv -f "${pending}.planner" "${RUN_DIR}/params_planner.yaml"
      fi
      if [[ ! -f "${RUN_DIR}/params_executor.yaml" ]] &&
        timeout 3 ros2 param dump /nav_executor > "${pending}.executor" 2>&1 &&
        grep -q 'ros__parameters:' "${pending}.executor"; then
        mv -f "${pending}.executor" "${RUN_DIR}/params_executor.yaml"
      fi
      if [[ -s "${RUN_DIR}/params_planner.yaml" &&
            -s "${RUN_DIR}/params_executor.yaml" &&
            -s "${RUN_DIR}/topics.txt" ]]; then
        break
      fi
    fi
    sleep 2
  done
  rm -f "${pending}.nodes" "${pending}.topics" "${pending}.planner" "${pending}.executor"
}
capture_runtime_graph &
GRAPH_CAPTURE_PID=$!

# Keep the recorder in the foreground: Ctrl-C in this terminal is delivered to
# rosbag2 directly, which lets it flush the MCAP index before we copy the logs.
# A background recorder is unreliable under shell job control and can be left
# stopped/hung waiting for a terminal.
RECORDER=(ros2 bag record "${BAG_ARGS[@]}" --topics "${TOPICS[@]}")
# Optional bounded run for a dry run / short calibration: FIELD_RECORD_SECONDS=30.
if [[ -n "${FIELD_RECORD_SECONDS:-}" ]]; then
  RECORDER=(timeout -s INT "${FIELD_RECORD_SECONDS}" "${RECORDER[@]}")
fi
trap ':' INT
set +e
"${RECORDER[@]}" > "${RUN_DIR}/bag_record.log" 2>&1
BAG_RC=$?
set -e
trap - INT
# timeout(1) returns 124, and an interactive Ctrl-C commonly reaches rosbag2
# as 130. Both mean the same thing here: the recorder stopped and should be
# indexed/copied normally.
if [[ "${BAG_RC}" -eq 124 || "${BAG_RC}" -eq 130 ]]; then
  BAG_RC=0
fi
# The recorder is already stopped. Ignore further terminal interrupts while
# the graph snapshot worker exits and the remaining artifacts are collected.
trap '' INT TERM HUP
touch "${RUN_DIR}/recording_stopped.marker"
wait "${GRAPH_CAPTURE_PID}" || true
set +e

# Read the bag index first, but do not let a missing/bad bag skip the artifact
# copy below.
if [[ -f "${RUN_DIR}/bag/metadata.yaml" ]]; then
  ros2 bag info "${RUN_DIR}/bag" > "${RUN_DIR}/bag_info.txt" 2>&1 || true
else
  echo "Bag metadata missing; recording did not produce a usable bag." > "${RUN_DIR}/bag_info.txt"
fi
MESSAGE_COUNT="$(awk '/^Messages:/ {print $2; exit}' "${RUN_DIR}/bag_info.txt" 2>/dev/null)"

# Copy the evidence that is already on disk before optional ROS graph dumps,
# which can be slow or fail after the navigation node has exited.
find "${ROS_LOG_DIR:-${HOME}/.ros/log}" -maxdepth 3 -type f -newer "${RUN_DIR}/start.marker" -print0 2>/dev/null |
  while IFS= read -r -d '' file; do
    cp -f "${file}" "${RUN_DIR}/logs/$(basename "${file}")" 2>/dev/null || true
  done
find "${WORKSPACE}/.scratch" -maxdepth 1 -type f -newer "${RUN_DIR}/start.marker" \
  \( -name '*perf*.csv' -o -name 'minco*.csv' \) -print0 2>/dev/null |
  while IFS= read -r -d '' file; do
    cp -f "${file}" "${RUN_DIR}/csv/$(basename "${file}")" 2>/dev/null || true
  done

LOG_COUNT="$(find "${RUN_DIR}/logs" -maxdepth 1 -type f 2>/dev/null | wc -l)"
CSV_COUNT="$(find "${RUN_DIR}/csv" -maxdepth 1 -type f 2>/dev/null | wc -l)"
CONFIG_COUNT="$(find "${RUN_DIR}/config" -maxdepth 1 -type f 2>/dev/null | wc -l)"
{
  echo "finished=$(date -Is)"
  echo "bag=${RUN_DIR}/bag"
  echo "bag_rc=${BAG_RC}"
  echo "bag_messages=${MESSAGE_COUNT:-unknown}"
  echo "logs=${RUN_DIR}/logs (${LOG_COUNT} files)"
  echo "csv=${RUN_DIR}/csv (${CSV_COUNT} files)"
  echo "config=${RUN_DIR}/config (${CONFIG_COUNT} files)"
  echo "topics=${RUN_DIR}/topics.txt"
  echo "bag_info=${RUN_DIR}/bag_info.txt"
  echo
  echo "Analysis hints:"
  echo "  - compare /rog_map/layer_type and /rog_map/layer_height_delta at failure times"
  echo "  - compare /nav_executor/online_terrain_map with /nav_executor/online_global_map"
  echo "  - grep logs for 'Region status transition' and 'open set exhausted'"
  echo "  - grep logs for 'Narrow-passage continuation active'"
} > "${RUN_DIR}/README.txt"

if [[ ! -s "${RUN_DIR}/params_planner.yaml" || ! -s "${RUN_DIR}/params_executor.yaml" ]]; then
  echo "WARNING: live parameter capture missed one or both navigation nodes; check nodes.txt and config/." >&2
fi

if [[ -z "${MESSAGE_COUNT}" || "${MESSAGE_COUNT}" == "0" ]]; then
  echo "WARNING: bag has no messages; check that navigation was running while recording." >&2
fi
if [[ "${CSV_COUNT}" -eq 0 ]]; then
  echo "WARNING: no minco/rog_map CSV copied; check planner performance paths." >&2
fi
echo "Recorded run: ${RUN_DIR} (messages=${MESSAGE_COUNT:-unknown}, logs=${LOG_COUNT}, csv=${CSV_COUNT})"
trap - INT TERM HUP
exit "${BAG_RC}"
