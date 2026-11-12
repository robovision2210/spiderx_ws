#!/usr/bin/env bash
# S4d-M LIVE attempt (ONE free-base launch; the enabled B1 run installation: M55 gate True, M61 and M6.0-D
# False): the S4d shadow run script (tested) with the S4c level-2 live changes. The committed S4d driver in
# its default LIVE mode, scripts/s4/s4d_driver.py (no --shadow-rehearsal): the 111-goal plan, the close
# while moving and the zero-goal reconnect probe. No retry, no extra cycle, no recovery motion: a stop or
# any emergency response ends the attempt. Kept from S4c and the S4d shadow run:
#  1. the launch itself starts the M5.5 node (SHADOW: no action client, sends nothing; its evidence
#     in RUN/node); the launch's GUI is reniced to +19 and its node to -5;
#  2. once the node is up the run waits for the operator's pre-key checks (premotion_checks.sh, the
#     LIVE DISPATCH checks; settle_check.py --dispatch; one screenshot): go.flag starts the driver;
#     abort.flag, or no decision within PREMOTION_WAIT_S, stops with no key pressed;
#  3. one recorder outside the repository, without _action/status or /clock; captures before and
#     after; one screenshot after a completed script; the simulator server's own exit by strace.
# S4d changes (harness diffs in provenance/):
#  a. the environment must hold the fresh ENABLED run installation and not the existing workspace
#     installation; the imported gates must be exactly M55 True, M61 False, M6.0-D False, from the run
#     installation; the checked-out branch must be the recorded local enabling branch, never pushed,
#     differing from the disabled baseline by exactly the two-line patch; otherwise nothing is launched;
#  b. OD8: the watchdog (armed before this script) must be running, with at least the 90-minute
#     script cap + 15 min left, before the launch and again before the first key;
#  c. OD16: the disk rule on the bag's filesystem, before the launch and again just before recording;
#  d. the driver's and the recorder's pids are recorded (RUN/driver.pid, RUN/bag.pid) for cleanup.sh,
#     which stops only processes this run owns; the teleops left after the driver are recorded,
#     then stopped through the sessions the driver logged, not by a name pattern.
# Usage: s4d_live_run.sh <run dir> <bag dir>
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools; WS=/home/user/spiderx_ws
RUNINST=/root/spiderx_live_enabled_20261111T052817Z/install
BASE=9cc1f6844de11abc630387d4c6c441a623463094
PREMOTION_WAIT_S=${PREMOTION_WAIT_S:-1500}
SCRIPT_CAP_S=5400; AFTER_S=900          # OD8: the driver's 90-minute cap; stop, cleanup, restore, verification
source $EVT/env.sh
step() { echo "[$(date -u +%T)] $*" | tee -a $RUN/steps.log; }
mkdir -p $RUN/node $RUN/captures
refuse() { step "REFUSED before the launch (nothing launched, no key): $*"; step end; exit 1; }
wd_check() {
  local WP DL REM; WP=$(tr -dc '0-9' < $RUN/watchdog.pid 2>/dev/null); DL=$(tr -dc '0-9' < $RUN/watchdog_deadline_epoch.txt 2>/dev/null)
  if [ -z "$WP" ] || [ -z "$DL" ] || ! kill -0 "$WP" 2>/dev/null || ! tr '\0' ' ' < /proc/$WP/cmdline | grep -q "watchdog.sh $RUN"; then
    step "OD8 $1: FAIL: the watchdog is not running"; return 1; fi
  REM=$(( DL - $(date +%s) ))
  if [ "$REM" -ge $(( SCRIPT_CAP_S + AFTER_S )) ]; then step "OD8 $1: PASS: watchdog $WP has ${REM} s left >= ${SCRIPT_CAP_S} + ${AFTER_S} s"; return 0; fi
  step "OD8 $1: FAIL: watchdog $WP has ${REM} s left < ${SCRIPT_CAP_S} + ${AFTER_S} s"; return 1
}
disk_check() {
  local DL OUT RC; DL=$(tr -dc '0-9' < $RUN/watchdog_deadline_epoch.txt 2>/dev/null)
  OUT=$(python3 - "$(dirname "$BAG")" "$(( ${DL:-0} - $(date +%s) ))" <<'PYEOF'
import os, sys
st = os.statvfs(sys.argv[1]); avail = st.f_bavail * st.f_frsize
T = max(0, int(sys.argv[2]))   # recording-duration cap: the watchdog time left (7200 s minus the time used)
r = 2 * 30.8e6 / 60            # B/s: 2 x the highest observed 30.8 MB per wall minute (S4b live); a PLANNING ASSUMPTION
E = r * T
restore = 0.1e9                # restore reserve: measured restore evidence <= 11 KB, workspace build+install 6.2 MB; rounded up
need = max(5 * 2**30, 2 * E + restore)
print(f'{"PASS" if avail >= need else "FAIL"}: {avail} B free on {sys.argv[1]}; T {T} s; E = 2 x 30.8 MB/min '
      f'(planning assumption) x T = {E:.0f} B; need max(5 GiB, 2E + {restore:.0f} B) = {need:.0f} B')
sys.exit(0 if avail >= need else 1)
PYEOF
); RC=$?; step "OD16 $1: $OUT"; return $RC
}
for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do echo "$v=${!v}"; done > $RUN/env_run.txt
grep -q "/home/user/spiderx_ws/install" $RUN/env_run.txt && refuse "the existing workspace installation is in the environment"
grep -q "$RUNINST/spiderx_controller" $RUN/env_run.txt || refuse "the run installation $RUNINST is not in the environment"
G=$(cd / && python3 -c 'from spiderx_controller import m55_contract as a, m61_live_contract as b, m6_live_contract as c; print(a.M55_LOCOMOTION_DISPATCH_ENABLED, b.M61_LIVE_DISPATCH_ENABLED, c.LIVE_DISPATCH_ENABLED, a.__file__)')
step "gates (imported): $G"
[ "$(echo $G | cut -d' ' -f1-3)" = "True False False" ] || refuse "the imported gates are not exactly M55 True, M61 False, M6.0-D False"
case "$(echo $G | cut -d' ' -f4)" in $RUNINST/*) ;; *) refuse "the gates were not imported from the run installation";; esac
EBR=$(cat $(dirname "$RUN")/enabling_branch.txt 2>/dev/null)
CUR=$(cd $WS && git rev-parse --abbrev-ref HEAD)
NS=$(cd $WS && git diff --numstat $BASE HEAD | tr '\t' ' ' | sort | tr '\n' ';')
REMOTE=$(cd $WS && git for-each-ref "refs/remotes/*/$EBR" | wc -l)
step "enabling branch: recorded '${EBR:-none}', checked out '$CUR', diff vs $BASE: $NS remote-tracking refs: $REMOTE; worktree entries: $(cd $WS && git status --porcelain | wc -l)"
case "$EBR" in local/m55-s4d-m-*) ;; *) refuse "no recorded local/m55-s4d-m-* enabling branch";; esac
[ "$CUR" = "$EBR" ] || refuse "the checked-out branch is not the recorded enabling branch"
[ "$NS" = "1 1 src/spiderx_controller/spiderx_controller/m55_contract.py;1 1 src/spiderx_controller/test/m55_gate.py;" ] || refuse "the enabling commit is not exactly the two-line M55 patch"
[ "$REMOTE" = 0 ] || refuse "the enabling branch has a remote-tracking ref (it must never be pushed)"
[ -z "$(cd $WS && git status --porcelain)" ] || refuse "the worktree is not clean"
wd_check "before the launch" || refuse "OD8 time budget"
disk_check "before the launch" || refuse "OD16 disk"
step "launch"; bash $EVT/stage_start.sh $RUN | tee -a $RUN/steps.log
abort() { step "ABORT: $*"; bash $EVT/stage_stop.sh $RUN > /dev/null; step end; exit 1; }
bash $EVT/stage_wait_controllers.sh $RUN; RC=$?; step "controllers: exit $RC"
[ $RC = 0 ] || abort "controllers not active (Gazebo startup race?); relaunch is a new attempt"
sleep 3
# Host load (Phase C): the non-essential Gazebo GUI runs at the lowest priority, the locomotion node
# at a higher one. Thresholds unchanged.
GUI=$(pgrep -x -f "ign gazebo gui" | tr '\n' ' ')
[ -n "$GUI" ] && renice -n 19 -p $GUI > /dev/null
step "gazebo gui pid(s) $GUI reniced +19"
# The node is this launch's own (fortress_m55_walking.launch.py); no second node.
LP=$(cat $RUN/launch.pid)
for i in $(seq 1 400); do grep -q "locomotion node up" $RUN/launch.log && break; grep -q "process has died" $RUN/launch.log && break; sleep 1; done
grep -q "locomotion node up" $RUN/launch.log || abort "locomotion node not up (see launch.log)"
step "node up after ${i} s: $(grep -o 'locomotion node up[^\"]*' $RUN/launch.log | head -1 | cut -c1-160)"
NODE=$(pgrep -s $LP -f "spiderx_controller/m55_locomotion_node" | tr '\n' ' ')
[ -n "$NODE" ] && renice -n -5 -p $NODE > /dev/null
ps -eo pid,ppid,sid,ni,pcpu,args --forest | grep -E "ign gazebo|m55_locomotion" | grep -v grep | cut -c1-140 > $RUN/captures/priorities.txt
step "locomotion node pid(s) $NODE (launch session $LP) reniced -5"
sleep 3
# The operator's pre-key checks (plugin provenance, gain, rate, timestep, ownership with the node as the
# action's only client, DISPATCH mode with the gate True, settling, screenshot). No key until go.flag.
touch $RUN/premotion_wait.flag
step "pre-key checks: waiting for go.flag or abort.flag (at most $PREMOTION_WAIT_S s)"
for i in $(seq 1 $PREMOTION_WAIT_S); do { [ -f $RUN/go.flag ] || [ -f $RUN/abort.flag ]; } && break; sleep 1; done
[ -f $RUN/abort.flag ] && abort "pre-key checks failed: $(head -c 300 $RUN/abort.flag)"
[ -f $RUN/go.flag ] || abort "no pre-key decision within $PREMOTION_WAIT_S s"
step "pre-key checks passed: $(head -c 300 $RUN/go.flag)"
wd_check "before the first key" || abort "OD8 time budget before the first key (no key pressed)"
cap() { timeout 20 ros2 node info /spiderx_locomotion > $RUN/captures/$1_node_info.txt 2>&1
  timeout 20 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $RUN/captures/$1_action_info.txt 2>&1
  for t in /leg_trajectory_controller/joint_trajectory /cmd_vel /spiderx/locomotion/status; do echo "== $t"; timeout 20 ros2 topic info -v $t; done > $RUN/captures/$1_topics.txt 2>&1; }
step "captures before"; cap before
disk_check "before recording" || abort "OD16 disk before recording (no recording, no key pressed)"
setsid ros2 bag record -s sqlite3 --include-hidden-topics -o $BAG /spiderx/locomotion/status /joint_states \
  /spiderx/sim/world_poses /cmd_vel /leg_trajectory_controller/joint_trajectory /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/follow_joint_trajectory/_action/feedback > $RUN/bag_record.log 2>&1 &
# NOT _action/status: the locomotion node counts subscribers of the action's status topic as
# action clients (foreign_action_clients). Not /clock: sim time is in the joint-state stamps.
BP=$!; echo $BP > $RUN/bag.pid; sleep 4
( sleep 120; top -b -n1 -w 160 | head -22 > $RUN/captures/top_during.txt;
  ps -o pid,ni,pcpu,args -C ruby,python3,parameter_bridge,robot_state_publisher -w | cut -c1-160 >> $RUN/captures/top_during.txt ) &
step "driver"
bash -c 'echo $$ > "$0"; exec python3 "$@"' $RUN/driver.pid $WS/scripts/s4/s4d_driver.py --out $RUN/driver > $RUN/driver.out 2>&1; DRC=$?
step "driver exit $DRC: $(tail -1 $RUN/driver.out | cut -c1-200)"
sleep 3; step "captures after"; cap after
kill -INT -- -$BP 2>/dev/null; for i in $(seq 1 30); do pgrep -g $BP >/dev/null || break; sleep 0.5; done
step "bag stopped"
TS=$(grep '"teleop_started"' $RUN/driver/driver.jsonl 2>/dev/null | grep -o '"pid": *[0-9]*' | grep -o '[0-9]*$' | tr '\n' ' ')
TL=""; for T in $TS; do TL="$TL $(pgrep -a -s $T -f m55_teleop_keyboard | tr '\n' ';')"; done; TL=$(echo $TL)
step "teleop sessions the driver logged: ${TS:-none}; teleop processes left in them after the driver (its two q closes should leave none): ${TL:-none}"
for T in $TS; do pkill -KILL -s $T -f m55_teleop_keyboard 2>/dev/null; done
# One screenshot after a completed script (no goal in flight), only if the simulator is still up;
# it never delays cleanup by more than 15 s.
if [ "$DRC" = 0 ] && pgrep -g "$LP" > /dev/null; then
  env -i PATH=/usr/bin:/bin DISPLAY=:77 timeout 15 /usr/bin/import -display :77 -window root $RUN/captures/screenshot2_after_script.png > $RUN/captures/screenshot2.log 2>&1
  step "screenshot 2: exit $?"
else
  step "screenshot 2 skipped (driver exit $DRC or launch gone)"
fi
# The simulator server's own exit (the launch reports only its ruby wrapper's exit).
SP=$(cat $RUN/server.pid 2>/dev/null); STP=""
if [ -n "$SP" ] && [ "$(ps -o sid= -p $SP | tr -d ' ')" = "$LP" ]; then
  strace -e trace=none -o $RUN/server_exit_strace.txt -p $SP > $RUN/server_exit_strace.log 2>&1 &
  STP=$!; echo $STP > $RUN/strace.pid; sleep 2
  step "strace (signals only) attached to simulator server $SP: $(cat $RUN/server_exit_strace.log | head -2 | tr '\n' ' ')"
else
  step "simulator server pid unknown or not in this launch: no strace"
fi
step "stop launch"; bash $EVT/stage_stop.sh $RUN > /dev/null
if [ -n "$STP" ]; then
  for i in $(seq 1 20); do kill -0 $STP 2>/dev/null || break; sleep 0.5; done
  kill -0 $STP 2>/dev/null && { kill -INT $STP; step "strace still attached after the stop: detached"; }
  step "server exit (strace): $(tail -1 $RUN/server_exit_strace.txt 2>/dev/null)"
fi
step end
