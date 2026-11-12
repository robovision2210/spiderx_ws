#!/usr/bin/env bash
# S3 shadow pre-shadow checks (the Phase C pre-motion checks, adapted to the free-base launch): run
# while s3_shadow_run.sh waits after the node is up. Read-only: no goal, publisher or parameter
# change. Adaptations: the r2 overlay and its provenance; the launch's own locomotion node is
# expected (in this launch's session, no action client); the weld check is replaced by
# settle_check.py (free base); the simulator server pid is kept for the exit observation.
# Writes RUN/premotion/ and ends with "PREMOTION VERDICT: PASS" or "... FAIL: <items>".
# Usage: premotion_checks.sh <run dir>
RUN=$1; EVT=$(dirname "$RUN")/tools; D=$RUN/premotion; mkdir -p $D
source $EVT/env.sh
O=/root/spiderx_gz_overlay_ws_r2/install/gz_ros2_control/lib
strip() { sed 's/\x1b\[[0-9;]*m//g'; }
LP=$(cat $RUN/launch.pid)
F=""
# 1. The simulator process of THIS launch: the only process mapping the system plugin, in the
#    launch's session (stage_start.sh starts the launch with setsid: session id = launch pid).
{ echo "launch pid = session $LP"; ps -o pid,ppid,sid,pgid,etimes,args -s $LP | cut -c1-200; } > $D/launch_group.txt 2>&1
MAPPERS=$(grep -l "libgz_ros2_control-system\.so" /proc/[0-9]*/maps 2>/dev/null | cut -d/ -f3 | sort -n | tr '\n' ' ')
P=""; for p in $MAPPERS; do [ "$(ps -o sid= -p $p | tr -d ' ')" = "$LP" ] && P="$P $p"; done; P=$(echo $P)
{ echo "processes mapping libgz_ros2_control-system.so (system-wide): ${MAPPERS:-none}"
  echo "of these, in this launch's session: ${P:-none}"
  if [ -n "$P" ] && [ "$(echo $P | wc -w)" = 1 ] && [ "$(echo $MAPPERS | wc -w)" = 1 ]; then echo "PASS: one process maps the plugin and it belongs to this launch: $P"
  else echo "FAIL: expected exactly one mapping process, in this launch's session"; fi; } > $D/candidates.txt
grep -q "^PASS" $D/candidates.txt || F="$F process"
P=$(echo $P | awk '{print $1}')
[ -n "$P" ] && echo $P > $RUN/server.pid
if [ -n "$P" ]; then
  { echo "pid $P; ppid chain: $(ps -o ppid= -p $P | tr -d ' ') -> $(ps -o ppid= -p $(ps -o ppid= -p $P) | tr -d ' '); session $(ps -o sid= -p $P | tr -d ' ') (launch.pid $LP)"
    echo "exe: $(readlink /proc/$P/exe)"
    echo "cmdline: $(tr '\0' ' ' < /proc/$P/cmdline)"
    echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH in process env: $(tr '\0' '\n' < /proc/$P/environ | grep '^IGN_GAZEBO_SYSTEM_PLUGIN_PATH=' | cut -d= -f2-)"
    echo "GZ_SIM_SYSTEM_PLUGIN_PATH in process env: $(tr '\0' '\n' < /proc/$P/environ | grep '^GZ_SIM_SYSTEM_PLUGIN_PATH=' | cut -d= -f2-)"
  } > $D/sim_process.txt 2>&1
  cp /proc/$P/maps $D/maps_full.txt; grep -E "ros2_control|gz_hardware_plugins" $D/maps_full.txt > $D/maps_targets.txt
  # 2. Library provenance of the mapped objects (the Phase B check, unchanged).
  python3 - "$P" "$O" "$D" > $D/library_checks.txt 2>&1 <<'EOF'
import hashlib, os, sys
pid, ovl, d = sys.argv[1], sys.argv[2], sys.argv[3]
prov = {l.split()[2]: l.split()[1] for l in open('/root/spiderx_gz_overlay_ws_r2/OVERLAY_PROVENANCE.txt') if l.startswith('library ')}
rows = [l.split() for l in open(os.path.join(d, 'maps_full.txt'))]
ok = True
for name in ('libgz_ros2_control-system.so', 'libgz_hardware_plugins.so'):
    maps = [r for r in rows if len(r) >= 6 and r[5].endswith('/' + name)]
    paths = sorted({r[5] for r in maps}); deleted = [r for r in maps if len(r) > 6]
    want = os.path.join(ovl, name); st = os.stat(want)
    inodes = sorted({int(r[4]) for r in maps})
    h = hashlib.sha256(open(f'/proc/{pid}/map_files/{maps[0][0]}', 'rb').read()).hexdigest() if maps else None
    good = paths == [want] and not deleted and inodes == [st.st_ino] and h == prov.get(want)
    ok &= good
    print(f'{name}: mappings={len(maps)} paths={paths} deleted={len(deleted)} inode={inodes} '
          f'(overlay file inode {st.st_ino}) mapped-object sha256={h} provenance={prov.get(want)} '
          f'-> {"PASS" if good else "FAIL"}')
stock = [r[5] for r in rows if len(r) >= 6 and r[5].startswith('/opt/mm/') and
         any(r[5].endswith(n) for n in ('libgz_ros2_control-system.so', 'libgz_hardware_plugins.so',
                                         'libign_ros2_control-system.so'))]
print('stock gz_ros2_control libraries mapped:', sorted(set(stock)) or 'none'); ok &= not stock
other = sorted({r[5] for r in rows if len(r) >= 6 and 'ros2_control' in r[5] and r[5].startswith('/opt/mm/')})
print('toolchain ros2_control dependencies mapped (expected, not fallback):', other)
print('LIBRARY VERDICT:', 'PASS' if ok else 'FAIL')
EOF
fi
grep -q "LIBRARY VERDICT: PASS" $D/library_checks.txt 2>/dev/null || F="$F libraries"
# 3. Gain, controller period and physics step from the plugin's own log lines.
L=$RUN/launch.log
NG=$(grep -c "The position_proportional_gain has been set to: 1$" $L)
NALL=$(grep -c "position_proportional_gain has been set to" $L)
NERR=$(grep -c -E "must be (finite|a floating)|Could not initialize robot simulation interface" $L)
PER=$(grep -m1 "Desired controller update period" $L | strip)
{ echo "== plugin gain log entries"; grep -n "position_proportional_gain" $L | strip
  echo "exact 'has been set to: 1' lines: $NG; all 'has been set to' lines: $NALL"
  echo "== controller/physics period (plugin warning)"; echo "${PER:-none}"
  echo "== invalid-gain or init errors: $NERR"; } > $D/log_checks.txt
[ "$NG" = 1 ] && [ "$NALL" = 1 ] || F="$F gain_log"
[ "$NERR" = 0 ] || F="$F init_errors"
grep -q "update period (0.01 s) is slower than the gazebo simulation period (0.001 s)" <<< "$PER" || F="$F period"
# 4. Parameters.
GAIN=$(timeout 30 ros2 param get /gz_ros2_control position_proportional_gain 2>&1)
RATE=$(timeout 30 ros2 param get /controller_manager update_rate 2>&1)
printf '== ros2 param get /gz_ros2_control position_proportional_gain\n%s\n== ros2 param get /controller_manager update_rate\n%s\n' "$GAIN" "$RATE" > $D/params.txt
[ "$GAIN" = "Double value is: 1.0" ] || F="$F gain_param"
[ "$RATE" = "Integer value is: 100" ] || F="$F update_rate"
# 5. Command ownership: both controllers active, no action client, no command publisher, no
#    driver/teleop process yet; the locomotion node only as this launch's own process, with no
#    action client and no publisher towards the controller.
CTL=$(timeout 30 ros2 control list_controllers 2>&1 | strip)
ACT=$(timeout 30 ros2 action info /leg_trajectory_controller/follow_joint_trajectory 2>&1)
TOP=$(timeout 30 ros2 topic info /leg_trajectory_controller/joint_trajectory 2>&1)
FORB=$(pgrep -af "teleop|s[23]_driver" | grep -v -E "pgrep|premotion_checks" || true)
NODEP=$(pgrep -f "spiderx_controller/m55_locomotion_node" | tr '\n' ' '); NODEP=$(echo $NODEP)
NODESID=$(for p in $NODEP; do ps -o sid= -p $p | tr -d ' '; done | sort -u | tr '\n' ' '); NODESID=$(echo $NODESID)
NINFO=$(timeout 30 ros2 node info /spiderx_locomotion 2>&1)
NAC=$(sed -n '/Action Clients:/,$p' <<< "$NINFO" | grep -c "^    /" || true)
NPUB=$(sed -n '/Publishers:/,/Service Servers:/p' <<< "$NINFO" | grep -c "joint_trajectory" || true)
NSH=$(grep -c "locomotion node up in SHADOW mode (dispatch gate False)" $RUN/launch.log)
{ echo "== ros2 control list_controllers"; echo "$CTL"; echo "== ros2 action info"; echo "$ACT"
  echo "== ros2 topic info joint_trajectory"; echo "$TOP"; echo "== driver/teleop processes"; echo "${FORB:-none}"
  echo "== locomotion node processes: ${NODEP:-none} (session ${NODESID:-none}; launch session $LP)"
  echo "== ros2 node info /spiderx_locomotion"; echo "$NINFO"
  echo "action clients of the node: $NAC; publishers towards joint_trajectory: $NPUB; launch-log SHADOW lines: $NSH"
  echo "== ros2 node list"; timeout 30 ros2 node list 2>&1; } > $D/ownership.txt
[ "$(grep -cE "^(joint_state_broadcaster|leg_trajectory_controller) .*[[:space:]]active$" <<< "$CTL")" = 2 ] || F="$F controllers"
grep -q "^Action clients: 0$" <<< "$ACT" && grep -q "^Action servers: 1$" <<< "$ACT" || F="$F action_clients"
grep -q "^Publisher count: 0$" <<< "$TOP" || F="$F command_publishers"
[ -z "$FORB" ] || F="$F forbidden_process"
[ "$(echo $NODEP | wc -w)" = 1 ] && [ "$NODESID" = "$LP" ] || F="$F node_process"
[ "$NAC" = 0 ] && [ "$NPUB" = 0 ] && grep -q "^  Action Clients:" <<< "$NINFO" || F="$F node_clients"
[ "$NSH" -ge 1 ] || F="$F node_shadow_log"
# 6. (Phase C weld check removed: free base; settle_check.py checks the standing robot.)
if [ -z "$F" ]; then echo "PREMOTION VERDICT: PASS" | tee $D/verdict.txt
else echo "PREMOTION VERDICT: FAIL:$F" | tee $D/verdict.txt; fi
