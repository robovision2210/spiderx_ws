#!/usr/bin/env bash
# S4b SHADOW rehearsal (ONE free-base launch; all three gates False; no goal or cancel may reach the
# controller): the S4a shadow run script, adapted only for S4b (harness_vs_s4a_shadow.diff): the S4b
# driver with --shadow-rehearsal replaces the S4a driver (its scripted lease test kills the TELEOP with
# SIGKILL, never the simulator); after it, nothing but this script's process-level stop. As before:
#  1. the launch itself starts the M5.5 node (SHADOW: no action client, sends nothing; its evidence in
#     RUN/node): no separate node, no m61a preflight (fixed base only); the launch's node is reniced
#     to -5, as Phase C started its node;
#  3. once the node is up the run waits for the operator's pre-shadow checks (premotion_checks.sh,
#     settle_check.py, one screenshot): go.flag starts the S3 script; abort.flag, or no decision
#     within PREMOTION_WAIT_S, stops with no key pressed;
#  4. the S4b driver (scripts/s4/s4b_driver.py --shadow-rehearsal) replaces the S4a driver; its exit
#     code is kept; one screenshot after a completed script (driver exit 0, launch up, <= 15 s);
#  5. before the stop, strace (signals only, no system calls) observes the simulator server's own exit.
# Usage: s4b_shadow_run.sh <run dir> <bag dir>
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools; WS=/home/user/spiderx_ws
PREMOTION_WAIT_S=${PREMOTION_WAIT_S:-1500}
source $EVT/env.sh
step() { echo "[$(date -u +%T)] $*" | tee -a $RUN/steps.log; }
mkdir -p $RUN/node $RUN/captures
step "gates (imported): $(python3 -c 'from spiderx_controller import m55_contract as a, m61_live_contract as b, m6_live_contract as c; print(a.M55_LOCOMOTION_DISPATCH_ENABLED, b.M61_LIVE_DISPATCH_ENABLED, c.LIVE_DISPATCH_ENABLED)')"
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
# Change 1: the node is this launch's own (fortress_m55_walking.launch.py); no second node.
LP=$(cat $RUN/launch.pid)
for i in $(seq 1 400); do grep -q "locomotion node up" $RUN/launch.log && break; grep -q "process has died" $RUN/launch.log && break; sleep 1; done
grep -q "locomotion node up" $RUN/launch.log || abort "locomotion node not up (see launch.log)"
step "node up after ${i} s: $(grep -o 'locomotion node up[^\"]*' $RUN/launch.log | head -1 | cut -c1-160)"
NODE=$(pgrep -s $LP -f "spiderx_controller/m55_locomotion_node" | tr '\n' ' ')
[ -n "$NODE" ] && renice -n -5 -p $NODE > /dev/null
ps -eo pid,ppid,sid,ni,pcpu,args --forest | grep -E "ign gazebo|m55_locomotion" | grep -v grep | cut -c1-140 > $RUN/captures/priorities.txt
step "locomotion node pid(s) $NODE (launch session $LP) reniced -5"
sleep 3
# Change 3: the operator's pre-shadow checks (plugin provenance, gain, rate, timestep, ownership,
# SHADOW mode, settling, screenshot). No key is pressed until go.flag.
touch $RUN/premotion_wait.flag
step "pre-shadow checks: waiting for go.flag or abort.flag (at most $PREMOTION_WAIT_S s)"
for i in $(seq 1 $PREMOTION_WAIT_S); do { [ -f $RUN/go.flag ] || [ -f $RUN/abort.flag ]; } && break; sleep 1; done
[ -f $RUN/abort.flag ] && abort "pre-shadow checks failed: $(head -c 300 $RUN/abort.flag)"
[ -f $RUN/go.flag ] || abort "no pre-shadow decision within $PREMOTION_WAIT_S s"
step "pre-shadow checks passed: $(head -c 300 $RUN/go.flag)"
cap() { timeout 20 ros2 node info /spiderx_locomotion > $RUN/captures/$1_node_info.txt 2>&1
  timeout 20 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $RUN/captures/$1_action_info.txt 2>&1
  for t in /leg_trajectory_controller/joint_trajectory /cmd_vel /spiderx/locomotion/status; do echo "== $t"; timeout 20 ros2 topic info -v $t; done > $RUN/captures/$1_topics.txt 2>&1; }
step "captures before"; cap before
setsid ros2 bag record -s sqlite3 --include-hidden-topics -o $BAG /spiderx/locomotion/status /joint_states \
  /spiderx/sim/world_poses /cmd_vel /leg_trajectory_controller/joint_trajectory /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/follow_joint_trajectory/_action/feedback > $RUN/bag_record.log 2>&1 &
# NOT _action/status: the locomotion node counts subscribers of the action's status topic as
# action clients (foreign_action_clients). Not /clock: sim time is in the joint-state stamps.
BP=$!; sleep 4
( sleep 120; top -b -n1 -w 160 | head -22 > $RUN/captures/top_during.txt;
  ps -o pid,ni,pcpu,args -C ruby,python3,parameter_bridge,robot_state_publisher -w | cut -c1-160 >> $RUN/captures/top_during.txt ) &
step "driver"; python3 $WS/scripts/s4/s4b_driver.py --out $RUN/driver --shadow-rehearsal > $RUN/driver.out 2>&1; DRC=$?; step "driver exit $DRC: $(tail -1 $RUN/driver.out | cut -c1-200)"
sleep 3; step "captures after"; cap after
kill -INT -- -$BP 2>/dev/null; for i in $(seq 1 30); do pgrep -g $BP >/dev/null || break; sleep 0.5; done
step "bag stopped"; pkill -f m55_teleop_keyboard 2>/dev/null
# Phase C change 3: one screenshot after a completed script (no goal in flight), only if the
# simulator is still up; it never delays cleanup by more than 15 s.
if [ "$DRC" = 0 ] && pgrep -g "$LP" > /dev/null; then
  env -i PATH=/usr/bin:/bin DISPLAY=:77 timeout 15 /usr/bin/import -display :77 -window root $RUN/captures/screenshot2_after_script.png > $RUN/captures/screenshot2.log 2>&1
  step "screenshot 2: exit $?"
else
  step "screenshot 2 skipped (driver exit $DRC or launch gone)"
fi
# Change 5: the simulator server's own exit (the launch reports only its ruby wrapper's exit).
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
