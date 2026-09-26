#!/usr/bin/env bash
# M3 validation: SIMULATION-ONLY single-leg (front_left) forward and inverse kinematics.
#
# Run from the workspace root after building and sourcing:
#   colcon build --symlink-install && source install/setup.bash
#   ./scripts/validate_m3_kinematics.sh             # static checks only (no simulator)
#   ./scripts/validate_m3_kinematics.sh --runtime   # + launch, FK vs TF/Gazebo, IK targets, report
#   ./scripts/validate_m3_kinematics.sh --runtime --headless
#
# Static: modules import, URDF-derived leg geometry + target config (validate_controller_config),
# the M3 FK/IK unit tests, invalid target configs REFUSED before anything is sent, no hardware /
# wheel / diff-drive / fake odom / cmd_vel / mechaprime logic in the M3 files.
# Runtime: controlled Fortress (fortress_posture_hold.launch.py = M1 + ground-truth pose bridge),
# both controllers active, exactly one /joint_states publisher, validate_leg_kinematics (FK vs TF
# and Gazebo, negative targets rejected without commanding, safe IK targets reached and returned),
# JSON report checked, clean shutdown with no leftover processes.
#
# Single-leg simulation kinematics validation. Not walking, balance, locomotion or hardware.
set -u
cd "$(dirname "$0")/.."

runtime=0; headless=false
for arg in "$@"; do
  case "$arg" in
    --runtime) runtime=1 ;;
    --headless) headless=true ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg"; exit 2 ;;
  esac
done

fail=0
pass() { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
m3_files=(src/spiderx_controller/spiderx_controller/leg_kinematics.py
          src/spiderx_controller/spiderx_controller/kinematics_targets.py
          src/spiderx_controller/spiderx_controller/kinematics_validation.py
          src/spiderx_controller/scripts/validate_leg_kinematics
          src/spiderx_controller/config/m3_kinematics_targets.yaml)

echo "== Packages, installed files and imports"
for p in spiderx_controller spiderx_bringup spiderx_description tf2_ros tf2_msgs ros_gz_bridge; do
  ros2 pkg prefix "$p" > /dev/null 2>&1 && pass "$p" || bad "$p not found"
done
prefix=$(ros2 pkg prefix spiderx_controller 2>/dev/null)
for f in share/spiderx_controller/config/m3_kinematics_targets.yaml \
         lib/spiderx_controller/validate_leg_kinematics; do
  [ -e "$prefix/$f" ] && pass "installed $f" || bad "not installed: $f (rebuild)"
done
if python3 -c "import spiderx_controller.leg_kinematics, spiderx_controller.kinematics_targets, \
spiderx_controller.kinematics_validation" 2> "$tmp/imp"; then
  pass "M3 modules import"
else
  bad "M3 modules do not import:"; cat "$tmp/imp"
fi

echo "== URDF-derived geometry and target config"
if out=$(ros2 run spiderx_controller validate_controller_config 2>&1); then
  pass "validate_controller_config (includes the M3 geometry and target checks)"
  grep -o 'M3 .*' <<< "$out" | cut -c1-150 | sed 's/^/         /'
else
  bad "validate_controller_config:"; echo "$out" | grep FAIL
fi

echo "== M3 FK/IK unit tests"
if PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
     src/spiderx_controller/test/test_leg_kinematics.py \
     src/spiderx_controller/test/test_kinematics_targets.py > "$tmp/pytest" 2>&1; then
  pass "$(tail -1 "$tmp/pytest")"
else
  bad "unit tests failed:"; tail -30 "$tmp/pytest"
fi

echo "== Forbidden additions"
if grep -nE 'rplidar|ydlidar|spiderx_firmware|/dev/tty|serial_port' "${m3_files[@]}"; then
  bad "M3 files reference a hardware driver"
else
  pass "M3 files reference no hardware driver"
fi
if grep -niE 'mechaprime|diff_drive|diff-drive|wheel_|/cmd_vel|/odom|nav_msgs' "${m3_files[@]}"; then
  bad "M3 files contain wheel / diff-drive / cmd_vel / odom / mechaprime logic"
else
  pass "no wheel, diff-drive, /cmd_vel, /odom or mechaprime logic in M3 files"
fi

echo "== Invalid target configs are refused before anything is sent (no simulator needed)"
cfg=src/spiderx_controller/config/m3_kinematics_targets.yaml
refused() {  # $1 = description, $2 = config file, $3 = expected reason (regex)
  ros2 run spiderx_controller validate_leg_kinematics --config "$2" \
    --output "$tmp/should_not_exist.json" > "$tmp/ref.out" 2>&1
  rc=$?
  if [ $rc -eq 2 ] && grep -qE "REFUSED.*($3)" "$tmp/ref.out" \
     && [ ! -e "$tmp/should_not_exist.json" ]; then
    pass "$1 refused ($(grep -m1 -o 'REFUSED.*' "$tmp/ref.out" | cut -c1-90))"
  else
    bad "$1 was not refused (exit $rc)"; cat "$tmp/ref.out"
  fi
}
sed 's/    offset_m: \[0.0, 0.0, 0.010\]/    offset_m: [0.0, 0.0, -0.010]/' "$cfg" > "$tmp/ground.yaml"
refused "safe target pushing 10 mm into the ground" "$tmp/ground.yaml" "min_lift_m"
sed 's/^leg: front_left/leg: rear_right/' "$cfg" > "$tmp/leg.yaml"
refused "unsupported leg rear_right" "$tmp/leg.yaml" "unsupported leg"
sed 's/^frame: base_link/frame: odom/' "$cfg" > "$tmp/frame.yaml"
refused "frame odom" "$tmp/frame.yaml" "frame must be"
sed 's/^simulation_only: true/simulation_only: false/' "$cfg" > "$tmp/sim.yaml"
refused "simulation_only: false" "$tmp/sim.yaml" "simulation_only"

if [ "$runtime" = 1 ]; then
  echo "== Runtime: ros2 launch spiderx_bringup fortress_posture_hold.launch.py headless:=$headless"
  setsid ros2 launch spiderx_bringup fortress_posture_hold.launch.py headless:=$headless \
    > "$tmp/launch.log" 2>&1 &
  lpid=$!
  if timeout 240 bash -c 'until ros2 control list_controllers 2>/dev/null \
        | grep -q "leg_trajectory_controller.*active"; do sleep 3; done'; then
    sleep 3
    ctrls=$(ros2 control list_controllers 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g')
    for c in joint_state_broadcaster leg_trajectory_controller; do
      grep -qE "^$c .* active" <<< "$ctrls" && pass "$c active" || bad "$c not active"
    done
    n_pub=$(ros2 topic info /joint_states 2>/dev/null | awk '/Publisher count/ {print $3}')
    [ "$n_pub" = 1 ] && pass "/joint_states has exactly 1 publisher" \
      || bad "/joint_states publisher count = $n_pub (expected 1)"
    grep -qiE 'rplidar|ydlidar|serial' <<< "$(ros2 node list 2>/dev/null)" \
      && bad "hardware driver node running" || pass "no hardware driver node running"

    report="$tmp/leg_kinematics_report.json"
    ros2 run spiderx_controller validate_leg_kinematics --output "$report" > "$tmp/kin" 2>&1
    rc=$?
    sed 's/^/    | /' "$tmp/kin"
    if [ -f "$report" ]; then
      if python3 - "$report" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
need = ['simulation_only', 'outcome', 'fk_verified', 'ik_verified', 'failures', 'leg', 'frame',
        'geometry', 'tolerances', 'reference_tip_m', 'controller_checks',
        'joint_state_publishers', 'fk_checks', 'negative_tests', 'ik_targets', 'limitations']
missing = [k for k in need if k not in r or r[k] is None]
assert r.get('simulation_only') is True
assert r.get('leg') == 'front_left' and r.get('frame') == 'base_link'
allowed = {'Single-leg FK verified against the current URDF/TF/Gazebo model.',
           'Single-leg IK verified for documented reachable, joint-safe simulation targets.',
           'M3 kinematics validation not verified.'}
assert set(r['outcome']) <= allowed, r['outcome']
for c in r.get('fk_checks', []):
    for k in ('vs_tf', 'vs_gazebo'):
        assert 'link_origin_error_m' in c[k] or not c[k]['passed']
for t in r.get('ik_targets', []):
    if t['passed']:
        assert 'target_error_m' in t and t['returned_to_start']
for n in r.get('negative_tests', []):
    if n['passed']:
        assert n['goals_sent'] == 0
if missing:
    print('missing report fields:', missing); sys.exit(1)
PY
      then pass "report JSON complete (simulation_only, geometry, FK checks, IK targets, negatives)"
      else bad "report JSON incomplete or inconsistent"
      fi
      mkdir -p log/m3_kinematics && cp "$report" log/m3_kinematics/latest_report.json
      echo "         report copied to log/m3_kinematics/latest_report.json"
    else
      bad "no report written"
    fi
    grep -q 'Single-leg FK verified' "$tmp/kin" && pass "Single-leg FK verified" \
      || bad "FK not verified"
    grep -q 'Single-leg IK verified' "$tmp/kin" && pass "Single-leg IK verified" \
      || bad "IK not verified"
    [ $rc -eq 0 ] || bad "validate_leg_kinematics exit code $rc"
  else
    bad "controllers not active within 240 s; last launch output:"; tail -25 "$tmp/launch.log"
  fi
  kill -INT -- -"$lpid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$lpid" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$lpid" 2>/dev/null
  sleep 2
  left=$(pgrep -f 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' || true)
  [ -z "$left" ] && pass "clean shutdown: no leftover simulation/controller processes" \
    || { bad "leftover processes after shutdown:"; ps -o pid,cmd -p $left; }
fi

echo
echo "Scope: single-leg (front_left) simulation kinematics validation."
echo "Not walking, gait, balance, locomotion, odometry or hardware validation."
if [ "$fail" = 0 ]; then echo "All M3 checks passed."; else echo "Some M3 checks FAILED."; fi
exit "$fail"
