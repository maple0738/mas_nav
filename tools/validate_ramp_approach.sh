#!/usr/bin/env bash
# Compare the same binary/configuration with the new ramp chain off (A) or on (B).
set -eo pipefail
NAV_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_DIR=$(cd -- "$NAV_DIR/../../simulation/mas_nav_sim" && pwd)
NAV_INSTALL=${MAS_NAV_SIM_INSTALL:-$NAV_DIR/../../../environment/build/ros/jazzy/navigation/active/mas_nav/install}
VARIANT=${1:-B}
[[ "$VARIANT" == A || "$VARIANT" == B ]] || { echo 'Usage: validate_ramp_approach.sh [A|B] [output-directory]' >&2; exit 2; }
RUN_DIR=${2:-${TMPDIR:-/tmp}/navigation-ramp-$VARIANT-$(date +%Y%m%d-%H%M%S)}
mkdir -p "$RUN_DIR"
RUN_DIR=$(cd -- "$RUN_DIR" && pwd)
printf 'Validation artifacts: %s\n' "$RUN_DIR"
export ROS_LOG_DIR="$RUN_DIR/ros" ROS_HOME="$RUN_DIR/ros-home" GZ_HOMEDIR="$RUN_DIR/gazebo"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$GZ_HOMEDIR"
# DDS/Gazebo require local sockets. Record a blocked run instead of hanging.
python3 - "$RUN_DIR" <<'PY'
import json, socket, sys
from pathlib import Path
try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM):
        pass
except OSError as exc:
    (Path(sys.argv[1])/'status.json').write_text(json.dumps({'status':'blocked','reason':str(exc)},indent=2)+'\n')
    raise SystemExit('Cannot start DDS/Gazebo: '+str(exc))
PY
source /opt/ros/jazzy/setup.bash
source "$NAV_INSTALL/setup.bash"
source "$NAV_DIR/install/setup.bash"
export AMENT_PREFIX_PATH="$NAV_INSTALL/mas_nav_sim:$NAV_INSTALL/mas2027_robot_description:$AMENT_PREFIX_PATH"
python3 - "$RUN_DIR" "$VARIANT" <<'PY'
import hashlib, json, shutil, sys
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
out=Path(sys.argv[1]); variant=sys.argv[2]; package='mas2027_nav_executor'
original=Path(get_package_prefix(package)); share=Path(get_package_share_directory(package))
prefix=out/'config-prefix'; overlay=prefix/'share'/package
shutil.copytree(share,overlay)
index=prefix/'share/ament_index/resource_index/packages'; index.mkdir(parents=True)
(index/package).write_text('')
(prefix/'lib').symlink_to(original/'lib',target_is_directory=True)
for name,key in [('planner_params.yaml','planner'),('region_control.yaml','region_control')]:
    path=overlay/'config'/name; data=yaml.safe_load(path.read_text()); params=data['/**']['ros__parameters']
    if key=='planner': params[key]['directional_terrain']['enable']=variant=='B'
    else: params[key]['slope']['front_entry']['enable']=variant=='B'
    path.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True))
files=[original/'lib'/package/(package+'_node'),*sorted((overlay/'config').glob('*.yaml'))]
(out/'provenance.json').write_text(json.dumps({'variant':variant,'source_prefix':str(original),
    'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}},indent=2)+'\n')
PY
export AMENT_PREFIX_PATH="$RUN_DIR/config-prefix:$AMENT_PREFIX_PATH"
export ROS_DOMAIN_ID=${MAS_NAV_SIM_DOMAIN_ID:-178}
export MAS_NAV_SIM_DOMAIN_ID=$ROS_DOMAIN_ID
exec 9>"/tmp/mas_nav_ramp_${UID}_${ROS_DOMAIN_ID}.lock"
flock -n 9
launch_pid=
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 -- "-$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for attempt in {1..50}; do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -TERM -- "-$launch_pid" 2>/dev/null || true
      sleep 2
      kill -KILL -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" || true
  fi
}
trap cleanup EXIT INT TERM
setsid ros2 launch mas_nav_sim navigation_sim.launch.py gui:=false rviz:=false >"$RUN_DIR/launch.log" 2>&1 &
launch_pid=$!
# Keep the map for round 2: only this round matches the 122.48 s reference.
for round in 1 2; do
  for direction in outbound return; do
    if [[ "$direction" == outbound ]]; then goals='[[25.6,6.45]]'; else goals='[[3.4,9.5]]'; fi
    python3 "$SIM_DIR/scripts/validate_base_transit.py" --goals "$goals" \
      --output "$RUN_DIR/$direction-$round" --timeout 360 --require-slope --require-spin \
      --slope-only --record-maps >"$RUN_DIR/$direction-$round.log" 2>&1
  done
done
python3 - "$RUN_DIR" <<'PY'
import csv,json,sys
from pathlib import Path
out=Path(sys.argv[1]); legs={}
for directory in sorted(out.glob('*-[12]')):
    result=json.loads((directory/'results.json').read_text())
    if isinstance(result,dict): result=result.get('results',[result])
    rows=list(csv.DictReader((directory/'telemetry.csv').open()))
    longest=0.; start=None; previous=None
    for r in rows:
        t=float(r['sim_time']); x=float(r['x']); y=float(r['y'])
        waiting=16<=x<=20 and 7<=y<=10 and float(r['speed'])<.05 and float(r['goal_error'])>.5
        if waiting:
            if start is None or previous is None or t-previous>.25: start=t
            longest=max(longest,t-start)
        else: start=None
        previous=t
    legs[directory.name]={'elapsed_sim_s':sum(v['elapsed_sim_s'] for v in result),
        'passed':all(v['passed'] for v in result),'central_highland_max_wait_s':longest}
summary={'legs':legs,'round_trip_sim_s':{str(i):sum(legs[f'{d}-{i}']['elapsed_sim_s'] for d in ['outbound','return']) for i in [1,2]},
    'reference_warm_round_trip_s':122.48,'geometry_acceptance':'Review paths.jsonl and recorded maps; no geometric pass is inferred from timing.'}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(out/'status.json').write_text('{"status":"completed"}\n')
print(json.dumps(summary,indent=2))
PY
printf 'Validation artifacts: %s\n' "$RUN_DIR"
