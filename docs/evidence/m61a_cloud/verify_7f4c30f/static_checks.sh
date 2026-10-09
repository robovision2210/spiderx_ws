#!/bin/bash
# Offline static/configuration checks, run once. No simulator, no ROS graph required.
EV="$1"; OUT="$EV/static"
source /home/user/spiderx_ws/spiderx_evidence/m61a_phase1_20261009T075335Z/tools/env.sh
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd /home/user/spiderx_ws
run() {  # name, command...
  local name="$1"; shift
  local t0=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  "$@" < /dev/null > "$OUT/$name.log" 2>&1
  local rc=$?
  printf '%s\t%s\t%s\t%s\n' "$name" "$rc" "$t0" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$OUT/summary.tsv"
  echo "$name rc=$rc"
}
printf 'name\texit\tstart_utc\tend_utc\n' > "$OUT/summary.tsv"
{ echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=${RMW_IMPLEMENTATION:-default}"; } > "$OUT/env.txt"
run validate_fortress_static ./scripts/validate_fortress.sh
run validate_m1_static ./scripts/validate_m1_control.sh
run validate_m2_static ./scripts/validate_m2_posture.sh
run validate_m3_static ./scripts/validate_m3_kinematics.sh
run validate_m4_static ./scripts/validate_m4_all_leg_ik.sh
run m61a_clearance_check_config ros2 run spiderx_controller m61a_clearance --check-config
run m6_gait_replay_dry_run ros2 run spiderx_controller m6_gait_replay.py --dry-run --no-write
run m6_live_playback_dry_run ros2 run spiderx_controller m6_live_playback --dry-run --no-write
run m61a_observe_interface_only ros2 run spiderx_controller m61a_observe_fixed_base --interface-only
run m6_gait_replay_live_refused ros2 run spiderx_controller m6_gait_replay.py --live
run m6_live_playback_live_refused ros2 run spiderx_controller m6_live_playback --live
