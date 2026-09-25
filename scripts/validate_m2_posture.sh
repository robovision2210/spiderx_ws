#!/usr/bin/env bash
# M2 validation: SIMULATION-ONLY CAD neutral posture hold in Gazebo Fortress.
#
# Run from the workspace root after building and sourcing:
#   colcon build --symlink-install && source install/setup.bash
#   ./scripts/validate_m2_posture.sh             # static checks only (no simulator)
#   ./scripts/validate_m2_posture.sh --runtime   # + launch, posture-hold test, report checks
#   ./scripts/validate_m2_posture.sh --runtime --headless
#
# Static: packages and installed files, M2 posture config vs URDF (validate_controller_config),
# launch files parse, invalid posture configs are REFUSED before anything is sent, no hardware /
# wheel / diff-drive / cmd_vel / odom logic in the M2 files.
# Runtime: controllers active, exactly one /joint_states publisher, three joints displaced with
# the M1 tool so the trajectory exercises motion, run_posture_hold_test.py, the JSON report is
# complete, clean shutdown with no leftover processes.
#
# Simulation-only posture hold. Not balance, walking, IK or hardware validation.
set -u
cd "$(dirname "$0")/.."

runtime=0; headless=false
for arg in "$@"; do
  case "$arg" in
    --runtime) runtime=1 ;;
    --headless) headless=true ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg"; exit 2 ;;
  esac
done

fail=0
pass() { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
m2_files=(src/spiderx_controller/config/m2_simulation_postures.yaml
          src/spiderx_controller/spiderx_controller/posture_config.py
          src/spiderx_controller/spiderx_controller/posture_metrics.py
          src/spiderx_controller/scripts/run_posture_hold_test.py
          src/spiderx_bringup/launch/fortress_posture_hold.launch.py)

echo "== Packages and installed files"
for p in spiderx_controller spiderx_bringup gz_ros2_control ros_gz_bridge tf2_msgs; do
  ros2 pkg prefix "$p" > /dev/null 2>&1 && pass "$p" || bad "$p not found"
done
ctrl_share=$(ros2 pkg prefix spiderx_controller 2>/dev/null)
for f in share/spiderx_controller/config/m2_simulation_postures.yaml \
         lib/spiderx_controller/run_posture_hold_test.py; do
  [ -e "$ctrl_share/$f" ] && pass "installed $f" || bad "not installed: $f (rebuild)"
done
bring_share=$(ros2 pkg prefix spiderx_bringup 2>/dev/null)
[ -e "$bring_share/share/spiderx_bringup/launch/fortress_posture_hold.launch.py" ] \
  && pass "installed fortress_posture_hold.launch.py" || bad "fortress_posture_hold.launch.py not installed"

echo "== Static configuration"
if out=$(ros2 run spiderx_controller validate_controller_config 2>&1); then
  pass "controller configs, <ros2_control> and M2 posture config match the URDF"
  grep -o 'M2 posture [a-z_]* (simulation_only)[^;]*' <<< "$out" | sed 's/^/         /'
else
  bad "validate_controller_config:"; echo "$out" | grep FAIL
fi
ros2 launch spiderx_bringup fortress_posture_hold.launch.py --show-args > /dev/null 2>&1 \
  && pass "ros2 launch spiderx_bringup fortress_posture_hold.launch.py --show-args" \
  || bad "fortress_posture_hold.launch.py --show-args"
if grep -nE 'rplidar|ydlidar|spiderx_firmware|/dev/tty|serial_port' "${m2_files[@]}"; then
  bad "M2 files reference a hardware driver"
else
  pass "M2 files reference no hardware driver"
fi
if grep -niE 'mechaprime|diff_drive|diff-drive|wheel_|/cmd_vel|/odom' "${m2_files[@]}"; then
  bad "M2 files contain wheel / diff-drive / cmd_vel / odom / mechaprime logic"
else
  pass "no wheel, diff-drive, /cmd_vel, /odom or mechaprime logic in M2 files"
fi
if grep -qE "remappings=\[\(GZ_POSE_TOPIC, ROS_POSE_TOPIC\)\]" \
     src/spiderx_bringup/launch/fortress_posture_hold.launch.py \
   && ! grep -qE "ROS_POSE_TOPIC = '/tf'" src/spiderx_bringup/launch/fortress_posture_hold.launch.py; then
  pass "ground-truth pose bridge is remapped away from /tf"
else
  bad "ground-truth pose bridge remapping"
fi

echo "== Invalid posture configs are refused before anything is sent (no simulator needed)"
cfg=src/spiderx_controller/config/m2_simulation_postures.yaml
refused() {  # $1 = description, $2 = config file
  ros2 run spiderx_controller run_posture_hold_test.py --config "$2" \
    --output "$tmp/should_not_exist.json" > "$tmp/ref.out" 2>&1
  rc=$?
  if [ $rc -eq 2 ] && grep -q REFUSED "$tmp/ref.out" && [ ! -e "$tmp/should_not_exist.json" ]; then
    pass "$1 refused ($(grep -o 'REFUSED.*' "$tmp/ref.out" | cut -c1-90))"
  else
    bad "$1 was not refused (exit $rc)"; cat "$tmp/ref.out"
  fi
}
sed 's/      lf_foot_joint: 0.0/      lf_foot_joint: 0.6/' "$cfg" > "$tmp/over_limit.yaml"
refused "out-of-limit target lf_foot_joint 0.6" "$tmp/over_limit.yaml"
sed 's/      lf_foot_joint: 0.0/      lf_foot_joint: 0.40/' "$cfg" > "$tmp/margin.yaml"
refused "target inside URDF limit but inside the 0.05 margin (lf_foot_joint 0.40)" "$tmp/margin.yaml"
sed 's/      rr_foot_joint: 0.0/      left_wheel_joint: 0.0/' "$cfg" > "$tmp/unknown.yaml"
refused "unknown joint left_wheel_joint" "$tmp/unknown.yaml"
sed 's/^simulation_only: true/simulation_only: false/' "$cfg" > "$tmp/not_sim.yaml"
refused "simulation_only: false" "$tmp/not_sim.yaml"

if [ "$runtime" = 1 ]; then
  echo "== Runtime: ros2 launch spiderx_bringup fortress_posture_hold.launch.py headless:=$headless"
  setsid ros2 launch spiderx_bringup fortress_posture_hold.launch.py headless:=$headless \
    > "$tmp/launch.log" 2>&1 &
  lpid=$!
  if timeout 240 bash -c 'until ros2 control list_controllers 2>/dev/null \
        | grep -q "leg_trajectory_controller.*active"; do sleep 3; done'; then
    ctrls=$(ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g')
    for c in joint_state_broadcaster leg_trajectory_controller; do
      grep -qE "^$c .*active" <<< "$ctrls" && pass "$c active" || bad "$c not active"
    done
    n_pub=$(ros2 topic info /joint_states 2>/dev/null | awk '/Publisher count/ {print $3}')
    [ "$n_pub" = 1 ] && pass "/joint_states has exactly 1 publisher" \
      || bad "/joint_states publisher count = $n_pub (expected 1)"
    tf_pubs=$(ros2 topic info /tf -v 2>/dev/null | grep -c "Node name: spiderx_sim_ground_truth_bridge")
    [ "$tf_pubs" = 0 ] && pass "ground-truth bridge does not publish on /tf" \
      || bad "ground-truth bridge publishes on /tf"
    nodes=$(ros2 node list 2>/dev/null)
    grep -qiE 'rplidar|ydlidar|serial' <<< "$nodes" && bad "hardware driver node running" \
      || pass "no hardware driver node running"

    # Start away from the pose so the single 12-joint trajectory exercises motion (M1 tool).
    for j in rr_hip rf_thigh_joint lr_foot_joint; do
      ros2 run spiderx_controller test_one_joint.py --joint "$j" --target 0.2 > "$tmp/d" 2>&1 \
        && pass "displaced $j to +0.2 rad (M1 tool)" || { bad "displacing $j failed"; cat "$tmp/d"; }
    done
    report="$tmp/posture_hold_report.json"
    ros2 run spiderx_controller run_posture_hold_test.py --output "$report" > "$tmp/hold" 2>&1
    rc=$?
    cat "$tmp/hold" | sed 's/^/    | /'
    if [ -f "$report" ]; then
      if python3 - "$report" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
required = ['simulation_only', 'outcome', 'passed', 'failures', 'posture',
            'commanded_positions_rad', 'start_positions_rad', 'final_positions_rad',
            'final_abs_error_rad', 'controller_checks', 'joint_state_publishers', 'hold',
            'metrics', 'foot_contact', 'limitations', 'thresholds', 'action']
missing = [k for k in required if k not in r or r[k] is None]
m = r.get('metrics') or {}
for k in ('hold_duration_s', 'joints', 'body'):
    if m.get(k) is None:
        missing.append('metrics.' + k)
if m.get('body'):
    for k in ('frame', 'height_m', 'roll_rad', 'pitch_rad'):
        if k not in m['body']:
            missing.append('metrics.body.' + k)
if len(r.get('commanded_positions_rad') or {}) != 12:
    missing.append('12 commanded positions')
assert r.get('simulation_only') is True, 'simulation_only must be true'
assert r.get('outcome') in ('Simulation posture hold verified.',
                            'Simulation posture hold not verified.'), r.get('outcome')
assert (r['outcome'] == 'Simulation posture hold verified.') == bool(r['passed'])
assert r['foot_contact']['status'] == 'unavailable'
if missing:
    print('missing report fields:', missing); sys.exit(1)
PY
      then pass "report JSON complete (simulation_only, 12 targets, metrics, frame, contact status)"
      else bad "report JSON incomplete"
      fi
      moved=$(python3 -c "import json,sys;print(len(json.load(open(sys.argv[1]))['joints_moved_more_than_0_01_rad'] or []))" "$report" 2>/dev/null)
      [ "${moved:-0}" -ge 3 ] && pass "trajectory moved $moved displaced joints back" \
        || bad "trajectory moved ${moved:-0} joints (expected >= 3)"
      mkdir -p log/m2_posture_hold && cp "$report" log/m2_posture_hold/latest_report.json
      echo "         report copied to log/m2_posture_hold/latest_report.json"
    else
      bad "no report written"
    fi
    if [ $rc -eq 0 ]; then
      pass "run_posture_hold_test.py: $(tail -3 "$tmp/hold" | grep -o 'Simulation posture hold [a-z ]*\.')"
    else
      bad "run_posture_hold_test.py exit $rc: $(grep -o 'Simulation posture hold [a-z ]*\.' "$tmp/hold")"
    fi
  else
    bad "controllers not active within 240 s; last launch output:"; tail -25 "$tmp/launch.log"
  fi
  kill -INT -- -"$lpid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$lpid" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$lpid" 2>/dev/null
  sleep 2
  left=$(pgrep -f 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' | grep -v "^$$\$" || true)
  [ -z "$left" ] && pass "clean shutdown: no leftover simulation/controller processes" \
    || { bad "leftover processes after shutdown:"; ps -o pid,cmd -p $left; }
fi

echo
echo "Scope: simulation-only posture hold of the existing CAD neutral pose."
echo "Not balance, walking, IK, odometry or hardware validation."
if [ "$fail" = 0 ]; then echo "All M2 checks passed."; else echo "Some M2 checks FAILED."; fi
exit "$fail"
