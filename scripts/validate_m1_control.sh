#!/usr/bin/env bash
# M1 validation: SpiderX joint position control in Gazebo Fortress (gz_ros2_control).
#
# Run from the workspace root after building and sourcing:
#   colcon build --symlink-install && source install/setup.bash
#   ./scripts/validate_m1_control.sh             # static checks only (no simulator)
#   ./scripts/validate_m1_control.sh --runtime   # + launch fortress_control and move joints
#   ./scripts/validate_m1_control.sh --runtime --headless
#
# Static: control packages present, <ros2_control> block and controller YAML match the URDF,
# launch files parse, unsafe commands are refused before anything is sent.
# Runtime: controllers become active, exactly one /joint_states publisher with 12 joints, one
# joint moves to a target and back, three joints are displaced and the all-joint cad_neutral
# trajectory brings them back, and no hardware driver runs.
# This validates JOINT-POSITION CONTROL ONLY. It does not test standing, balance or walking.
set -u
cd "$(dirname "$0")/.."

runtime=0; headless=false
for arg in "$@"; do
  case "$arg" in
    --runtime) runtime=1 ;;
    --headless) headless=true ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg"; exit 2 ;;
  esac
done

fail=0
pass() { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

echo "== Packages"
for p in spiderx_controller spiderx_bringup gz_ros2_control controller_manager \
         joint_state_broadcaster joint_trajectory_controller control_msgs; do
  ros2 pkg prefix "$p" > /dev/null 2>&1 && pass "$p" || bad "$p not found (see docs/M1_JOINT_POSITION_CONTROL_GUIDE.md)"
done
lib="$(ros2 pkg prefix gz_ros2_control 2>/dev/null)/lib/libgz_ros2_control-system.so"
[ -f "$lib" ] && pass "gz_ros2_control Fortress system plugin: $lib" || bad "missing $lib"

echo "== Static configuration"
if out=$(ros2 run spiderx_controller validate_controller_config 2>&1); then
  pass "controller configs and <ros2_control> block match the URDF"
else
  bad "validate_controller_config:"; echo "$out" | grep FAIL
fi
for lf in "spiderx_bringup fortress_control.launch.py" "spiderx_controller controller.launch.py"; do
  # shellcheck disable=SC2086
  ros2 launch $lf --show-args > /dev/null 2>&1 && pass "ros2 launch $lf --show-args" \
    || bad "ros2 launch $lf --show-args"
done
if grep -nE 'rplidar|spiderx_firmware|/dev/tty|serial_port' \
     src/spiderx_bringup/launch/fortress_control.launch.py \
     src/spiderx_controller/launch/controller.launch.py; then
  bad "control launch files reference hardware"
else
  pass "control launch files reference no hardware driver"
fi

echo "== Unsafe commands are refused (no simulator needed)"
ros2 run spiderx_controller test_one_joint.py --joint left_wheel_joint > "$tmp/r1" 2>&1
[ $? -eq 2 ] && grep -q REFUSED "$tmp/r1" && pass "unknown joint 'left_wheel_joint' refused" \
  || bad "unknown joint was not refused"
ros2 run spiderx_controller test_one_joint.py --joint lf_foot_joint --target 0.6 > "$tmp/r2" 2>&1
[ $? -eq 2 ] && grep -q REFUSED "$tmp/r2" && pass "out-of-limit target (lf_foot_joint 0.6 > 0.436) refused" \
  || bad "out-of-limit target was not refused"

if [ "$runtime" = 1 ]; then
  echo "== Runtime: ros2 launch spiderx_bringup fortress_control.launch.py headless:=$headless"
  setsid ros2 launch spiderx_bringup fortress_control.launch.py headless:=$headless \
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
    names=$(timeout 30 ros2 topic echo /joint_states --once --field name 2>/dev/null | grep -m1 '^\[')
    [ "$(grep -o "'" <<< "$names" | wc -l)" = 24 ] && pass "/joint_states carries 12 joints" \
      || bad "/joint_states: $names"
    nodes=$(ros2 node list 2>/dev/null)
    grep -qiE 'rplidar|serial' <<< "$nodes" && bad "hardware driver node running" \
      || pass "no hardware driver node running"

    if ros2 run spiderx_controller test_one_joint.py --joint lf_hip --target 0.2 \
         --return-to-initial > "$tmp/one" 2>&1; then
      pass "single joint lf_hip -> +0.2 rad and back: $(grep 'Error magnitude' "$tmp/one" | sed 's/  */ /g')"
    else
      bad "single-joint test failed:"; cat "$tmp/one"
    fi
    for j in rr_hip rf_thigh_joint lr_foot_joint; do
      ros2 run spiderx_controller test_one_joint.py --joint "$j" --target 0.2 > "$tmp/d" 2>&1 \
        && pass "displaced $j to +0.2 rad" || { bad "displacing $j failed"; cat "$tmp/d"; }
    done
    if ros2 run spiderx_controller test_neutral_pose.py > "$tmp/neutral" 2>&1; then
      moved=$(grep -oE 'moved > 0.01 rad: [0-9]+' "$tmp/neutral" | grep -oE '[0-9]+$')
      if [ "${moved:-0}" -ge 3 ]; then
        pass "cad_neutral trajectory (all 12 joints): $moved joints moved, $(grep -oE 'Max \|error\| = [0-9.]+ rad' "$tmp/neutral")"
      else
        bad "neutral-pose test did not move the displaced joints"; cat "$tmp/neutral"
      fi
    else
      bad "neutral-pose test failed:"; cat "$tmp/neutral"
    fi
  else
    bad "controllers not active within 240 s; last launch output:"; tail -25 "$tmp/launch.log"
  fi
  kill -INT -- -"$lpid" 2>/dev/null; sleep 12; kill -KILL -- -"$lpid" 2>/dev/null
fi

echo
echo "Scope: joint-position control only. No standing, balance, gait, /cmd_vel or odometry."
if [ "$fail" = 0 ]; then echo "All M1 checks passed."; else echo "Some M1 checks FAILED."; fi
exit "$fail"
