#!/usr/bin/env bash
# Launch the maintained simulator with the workspace navigation overlay.
set -eo pipefail
NAV_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_DIR=$(cd -- "$NAV_DIR/../../simulation/mas_nav_sim" && pwd)
NAV_INSTALL=${MAS_NAV_SIM_INSTALL:-$NAV_DIR/../../../environment/build/ros/jazzy/navigation/active/mas_nav/install}
NAV_INSTALL=$(cd -- "$NAV_INSTALL" && pwd)
RUN_DIR=${MAS_NAV_SIM_RUN_DIR:-$NAV_DIR/.scratch/navigation-sim-$(date +%Y%m%d-%H%M%S)}
mkdir -p "$RUN_DIR"
RUN_DIR=$(cd -- "$RUN_DIR" && pwd)
export ROS_LOG_DIR="$RUN_DIR/ros" ROS_HOME="$RUN_DIR/ros-home" GZ_HOMEDIR="$RUN_DIR/gazebo"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$GZ_HOMEDIR"
export ROS_DOMAIN_ID=${MAS_NAV_SIM_DOMAIN_ID:-73}
export MAS_NAV_SIM_DOMAIN_ID=$ROS_DOMAIN_ID
exec 9>"/tmp/mas_nav_sim_${UID}_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo "MAS simulation is already running in ROS domain $ROS_DOMAIN_ID" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$NAV_INSTALL/setup.bash"
source "$NAV_DIR/install/setup.bash"
# Keep simulator and robot assets on the maintained installation. Only the
# navigation/dependency overlay comes from this workspace.
export AMENT_PREFIX_PATH="$NAV_INSTALL/mas_nav_sim:$NAV_INSTALL/mas2027_robot_description:$AMENT_PREFIX_PATH"
python3 - "$RUN_DIR" "$NAV_DIR/install" "$SIM_DIR" <<'PY'
import hashlib
import json
from pathlib import Path
import socket
import sys
from ament_index_python.packages import get_package_prefix, get_package_share_directory

out = Path(sys.argv[1])
prefix = Path(get_package_prefix('mas2027_nav_executor'))
expected = Path(sys.argv[2]) / 'mas2027_nav_executor'
if prefix.resolve() != expected.resolve():
    raise SystemExit(f'Wrong executor installation: {prefix}; expected {expected}')
share = Path(get_package_share_directory('mas2027_nav_executor'))
sim_prefix = Path(get_package_prefix('mas_nav_sim'))
sim_share = Path(get_package_share_directory('mas_nav_sim'))
sim_source = Path(sys.argv[3])
for installed, source in [
    (sim_share / 'launch/navigation_sim.launch.py', sim_source / 'launch/navigation_sim.launch.py'),
    (sim_share / 'launch/polarbear_model.py', sim_source / 'scripts/polarbear_model.py'),
]:
    if not installed.is_file() or installed.read_bytes() != source.read_bytes():
        raise SystemExit(f'Stale simulator installation: {installed}; rebuild the maintained installation')
files = [prefix / 'lib/mas2027_nav_executor/mas2027_nav_executor_node',
         *sorted((share / 'config').glob('*.yaml')),
         sim_share / 'launch/navigation_sim.launch.py', sim_share / 'launch/polarbear_model.py',
         sim_prefix / 'lib/libmas_polarbear_drive.so',
         Path(get_package_share_directory('mas2027_robot_description')) / 'urdf/mas2027_sentry.urdf']
(out / 'provenance.json').write_text(json.dumps({
    'executor_prefix': str(prefix),
    'simulator_prefix': str(sim_prefix),
    'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
}, indent=2) + '\n')
try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM):
        pass
except OSError as exc:
    (out / 'status.json').write_text(json.dumps({'status': 'blocked', 'reason': str(exc)}, indent=2) + '\n')
    raise SystemExit(f'Cannot start DDS/Gazebo: {exc}\nRun artifacts: {out}')
print(f'Executor: {prefix}\nSimulator: {sim_prefix}\nRun artifacts: {out}', flush=True)
PY
# Keep compact motion/heading/path evidence for the next comparison. Raw lidar
# clouds are intentionally excluded from this diagnostic recording.
recorder_pid=
cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    for attempt in {1..30}; do
      kill -0 "$recorder_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$recorder_pid" 2>/dev/null; then
      kill -TERM "$recorder_pid" 2>/dev/null || true
      for attempt in {1..20}; do
        kill -0 "$recorder_pid" 2>/dev/null || break
        sleep 0.1
      done
      kill -KILL "$recorder_pid" 2>/dev/null || true
    fi
    wait "$recorder_pid" || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 bag record --output "$RUN_DIR/motion" \
  /clock /Odometry /sim/chassis_odometry /goal_pose /opt_path \
  /nav_executor/chassis_cmd /nav_executor/region_status \
  /nav_executor/minco_path /nav_executor/global_plan >"$RUN_DIR/recorder.log" 2>&1 9>&- &
recorder_pid=$!
ros2 launch mas_nav_sim navigation_sim.launch.py "$@"
