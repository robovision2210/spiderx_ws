#!/usr/bin/env bash
# M4 validation: SIMULATION-ONLY all-leg kinematics and static multi-leg pose hold via IK.
#
# Run from the workspace root after building and sourcing:
#   colcon build --symlink-install && source install/setup.bash
#   ./scripts/validate_m4_all_leg_ik.sh             # static checks only (no simulator)
#   ./scripts/validate_m4_all_leg_ik.sh --runtime   # + launch, 3 poses in Gazebo, report checks
#   ./scripts/validate_m4_all_leg_ik.sh --runtime --headless
#
# Static: installed files and imports, validate_controller_config (M1-M3 configs unchanged),
# m4_pose_validation --check-only (all poses solvable and joint-safe), the M4 unit tests, invalid
# pose configs REFUSED for the expected reason before anything is sent, no hardware / wheel /
# diff-drive / fake odom / cmd_vel / mechaprime logic in the M4 files.
# Runtime: fortress_posture_hold.launch.py (M1 + Gazebo ground-truth bridge), both controllers
# active, one /joint_states publisher, m4_pose_validation (neutral_stance, crouch_10mm,
# lift_lf_15mm), JSON report checked and copied to log/m4_all_leg_ik/latest_report.json, clean
# shutdown with no leftover processes.
#
# Static poses in simulation only. Not walking, gait, balance, locomotion or hardware validation.
set -u
cd "$(dirname "$0")/.."

runtime=0; headless=false
for arg in "$@"; do
  case "$arg" in
    --runtime) runtime=1 ;;
    --headless) headless=true ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg"; exit 2 ;;
  esac
done

fail=0
pass() { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
m4_files=(src/spiderx_controller/spiderx_controller/m4_pose_targets.py
          src/spiderx_controller/spiderx_controller/m4_pose_validation.py
          src/spiderx_controller/scripts/m4_pose_validation
          src/spiderx_controller/config/m4_pose_targets.yaml)

echo "== Packages, installed files and imports"
for p in spiderx_controller spiderx_bringup spiderx_description tf2_ros tf2_msgs ros_gz_bridge; do
  ros2 pkg prefix "$p" > /dev/null 2>&1 && pass "$p" || bad "$p not found"
done
prefix=$(ros2 pkg prefix spiderx_controller 2>/dev/null)
for f in share/spiderx_controller/config/m4_pose_targets.yaml lib/spiderx_controller/m4_pose_validation; do
  [ -e "$prefix/$f" ] && pass "installed $f" || bad "not installed: $f (rebuild)"
done
if python3 -c "import spiderx_controller.m4_pose_targets, spiderx_controller.m4_pose_validation" \
     2> "$tmp/imp"; then
  pass "M4 modules import"
else
  bad "M4 modules do not import:"; cat "$tmp/imp"
fi

echo "== Configuration"
if out=$(ros2 run spiderx_controller validate_controller_config 2>&1); then
  pass "validate_controller_config (M1-M3 configs and URDF consistency)"
else
  bad "validate_controller_config:"; echo "$out" | grep FAIL
fi
if out=$(ros2 run spiderx_controller m4_pose_validation --check-only 2>&1); then
  pass "m4_pose_validation --check-only: $(grep -o "Poses .*validated" <<< "$out")"
else
  bad "M4 pose config does not validate:"; echo "$out"
fi

echo "== M4 unit tests"
if PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
     src/spiderx_controller/test/test_all_leg_kinematics.py \
     src/spiderx_controller/test/test_m4_pose_targets.py \
     src/spiderx_controller/test/test_m4_pose_validation.py > "$tmp/pytest" 2>&1; then
  pass "$(tail -1 "$tmp/pytest")"
else
  bad "unit tests failed:"; tail -30 "$tmp/pytest"
fi

echo "== Forbidden additions"
if grep -nE 'rplidar|ydlidar|spiderx_firmware|/dev/tty|serial_port' "${m4_files[@]}"; then
  bad "M4 files reference a hardware driver"
else
  pass "M4 files reference no hardware driver"
fi
if grep -niE 'mechaprime|diff_drive|diff-drive|wheel_|/cmd_vel|/odom|nav_msgs' "${m4_files[@]}"; then
  bad "M4 files contain wheel / diff-drive / cmd_vel / odom / mechaprime logic"
else
  pass "no wheel, diff-drive, /cmd_vel, /odom or mechaprime logic in M4 files"
fi

echo "== Invalid pose configs are refused before anything is sent (no simulator needed)"
cfg=src/spiderx_controller/config/m4_pose_targets.yaml
refused() {  # $1 = description, $2 = config file, $3 = expected reason (regex)
  ros2 run spiderx_controller m4_pose_validation --config "$2" \
    --output "$tmp/should_not_exist.json" > "$tmp/ref.out" 2>&1
  rc=$?
  if [ $rc -eq 2 ] && grep -qE "REFUSED.*($3)" "$tmp/ref.out" \
     && [ ! -e "$tmp/should_not_exist.json" ]; then
    pass "$1 refused ($(grep -m1 -o 'REFUSED.*' "$tmp/ref.out" | cut -c1-100))"
  else
    bad "$1 was not refused as expected (exit $rc)"; cat "$tmp/ref.out"
  fi
}
sed 's/^simulation_only: true/simulation_only: false/' "$cfg" > "$tmp/sim.yaml"
refused "simulation_only: false" "$tmp/sim.yaml" "simulation_only"
sed 's/^frame: base_link/frame: odom/' "$cfg" > "$tmp/frame.yaml"
refused "frame odom" "$tmp/frame.yaml" "frame must be"
python3 - "$cfg" "$tmp" <<'PY'
import sys, yaml
cfg, tmp = sys.argv[1], sys.argv[2]
d = yaml.safe_load(open(cfg))
m = yaml.safe_load(open(cfg)); del m['poses']['crouch_10mm']['offsets_m']['rear_left']
yaml.safe_dump(m, open(f'{tmp}/missing.yaml', 'w'))
u = yaml.safe_load(open(cfg)); u['safety']['max_offset_m'] = 0.5
u['poses']['crouch_10mm']['offsets_m']['rear_right'] = [0.0, 0.0, -0.30]
yaml.safe_dump(u, open(f'{tmp}/unreach.yaml', 'w'))
PY
refused "crouch_10mm missing rear_left" "$tmp/missing.yaml" "missing legs"
refused "crouch_10mm with an unreachable rear_right" "$tmp/unreach.yaml" "rear_right: unreachable"

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

    report="$tmp/all_leg_ik_report.json"
    ros2 run spiderx_controller m4_pose_validation --output "$report" > "$tmp/m4" 2>&1
    rc=$?
    sed 's/^/    | /' "$tmp/m4"
    if [ -f "$report" ]; then
      if python3 - "$report" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
need = ['simulation_only', 'outcome', 'fk_verified', 'ik_verified', 'hold_validated', 'failures',
        'joint_names', 'tolerances', 'hold_criteria', 'controller_checks',
        'joint_state_publishers', 'start_fk', 'poses', 'negative_poses', 'limitations']
missing = [k for k in need if k not in r or r[k] is None]
assert r['simulation_only'] is True and len(r['joint_names']) == 12
allowed = {'All-leg forward kinematics verified for the current URDF/TF/Gazebo model.',
           'All-leg inverse kinematics verified for documented, joint-safe, simulation-only '
           'static poses.',
           'Static multi-leg pose hold via IK validated in Gazebo (no walking, no gait, no '
           'hardware).',
           'M4 all-leg kinematics validation not verified.'}
assert set(r['outcome']) <= allowed, r['outcome']
for name, p in r['poses'].items():
    for k in ('action', 'fk_checks', 'tip_errors', 'body', 'expected_body_height_m', 'passed'):
        if k not in p:
            missing.append(f'poses.{name}.{k}')
assert all(not n['commanded'] for n in r['negative_poses'].values())
if missing:
    print('missing report fields:', missing); sys.exit(1)
PY
      then pass "report JSON complete (simulation_only, 12 joints, per-pose FK/tips/body, negatives)"
      else bad "report JSON incomplete or inconsistent"
      fi
      mkdir -p log/m4_all_leg_ik && cp "$report" log/m4_all_leg_ik/latest_report.json
      echo "         report copied to log/m4_all_leg_ik/latest_report.json"
    else
      bad "no report written"
    fi
    grep -q 'All-leg forward kinematics verified' "$tmp/m4" && pass "All-leg FK verified" \
      || bad "all-leg FK not verified"
    grep -q 'All-leg inverse kinematics verified' "$tmp/m4" && pass "All-leg IK verified" \
      || bad "all-leg IK not verified"
    grep -q 'Static multi-leg pose hold via IK validated' "$tmp/m4" \
      && pass "Static multi-leg pose hold validated" || bad "static pose hold not validated"
    [ $rc -eq 0 ] || bad "m4_pose_validation exit code $rc"
  else
    bad "controllers not active within 240 s; last launch output:"; tail -25 "$tmp/launch.log"
  fi
  kill -INT -- -"$lpid" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- -"$lpid" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$lpid" 2>/dev/null
  sleep 2
  # exclude this script's own ancestors (a calling shell may contain these words in its command line)
  anc=$(p=$$; while [ "${p:-1}" -gt 1 ]; do echo "$p"; p=$(ps -o ppid= -p "$p" | tr -d ' '); done)
  left=$(pgrep -f 'ign gazebo|gz sim|parameter_bridge|robot_state_publisher|controller_manager/spawner|ros2 launch spiderx_bringup' \
         | grep -vxF "$anc" || true)
  [ -z "$left" ] && pass "clean shutdown: no leftover simulation/controller processes" \
    || { bad "leftover processes after shutdown:"; ps -o pid,cmd -p $left; }
  if [ "$fail" != 0 ]; then       # keep the launch log for debugging (e.g. spawner timeouts)
    mkdir -p log/m4_all_leg_ik
    kept="log/m4_all_leg_ik/launch_log_$(date +%Y%m%d_%H%M%S).txt"
    cp "$tmp/launch.log" "$kept" && echo "  launch log preserved: $kept"
  fi
fi

echo
echo "Scope: four-leg static poses (neutral_stance, crouch_10mm, lift_lf_15mm) in simulation."
echo "Not walking, gait, balance, locomotion, odometry or hardware validation."
if [ "$fail" = 0 ]; then echo "All M4 checks passed."; else echo "Some M4 checks FAILED."; fi
exit "$fail"
