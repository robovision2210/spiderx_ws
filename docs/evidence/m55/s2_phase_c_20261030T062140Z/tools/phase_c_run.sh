#!/usr/bin/env bash
# S2 Phase C run: the frozen s2_rehearsal.sh (welded launch, preflight, node, bag, S2 driver,
# captures, clean stop) with three run-local changes (phase_c_run.diff):
#  1. after preflight READY the run waits for the operator's pre-motion checks: go.flag starts
#     the node and the driver; abort.flag, or no decision within PREMOTION_WAIT_S, stops with no goal;
#  2. the driver's exit code is kept (DRC);
#  3. one Gazebo screenshot after the script, only if the driver exited 0 and the launch is up.
# Usage: phase_c_run.sh <run dir> <bag dir>
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools
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
# Host load (attempt 3: a 0.8 s CPU-starvation stall of the node tripped the 0.5 s joint-state
# monitor while the stream itself never stopped). Thresholds unchanged; the non-essential
# Gazebo GUI ("ign gazebo gui": about 150 % CPU of 4 cores here, more than the server) runs at
# the lowest priority, the locomotion node at a higher one. Headless Gazebo is not available
# here (no EGL).
GUI=$(pgrep -x -f "ign gazebo gui" | tr '\n' ' ')
[ -n "$GUI" ] && renice -n 19 -p $GUI > /dev/null
ps -eo pid,ppid,ni,pcpu,args --forest | grep -E "ign gazebo" | grep -v grep | cut -c1-120 > $RUN/captures/priorities.txt
step "gazebo gui pid(s) $GUI reniced +19"
step "preflight"; timeout 120 ros2 run spiderx_controller m61a_observe_fixed_base --preflight --domain-id 0 --out $RUN/observer > $RUN/preflight.log 2>&1; RC=$?
step "preflight exit $RC $(grep -m1 -oE 'NOT READY|READY' $RUN/preflight.log)"
[ $RC = 0 ] || abort "preflight not READY"
# Phase C change 1: the operator's pre-motion checks (plugin provenance, gain, rate, timestep,
# ownership, weld, screenshot 1). Nothing that can send a goal exists until go.flag.
touch $RUN/premotion_wait.flag
step "pre-motion checks: waiting for go.flag or abort.flag (at most $PREMOTION_WAIT_S s)"
for i in $(seq 1 $PREMOTION_WAIT_S); do { [ -f $RUN/go.flag ] || [ -f $RUN/abort.flag ]; } && break; sleep 1; done
[ -f $RUN/abort.flag ] && abort "pre-motion checks failed: $(head -c 300 $RUN/abort.flag)"
[ -f $RUN/go.flag ] || abort "no pre-motion decision within $PREMOTION_WAIT_S s"
step "pre-motion checks passed: $(head -c 300 $RUN/go.flag)"
step "node"; setsid bash -c "source $EVT/env.sh; exec nice -n -5 ros2 run spiderx_controller m55_locomotion_node --evidence-dir $RUN/node" > $RUN/node.log 2>&1 &
echo $! > $RUN/node.pid
for i in $(seq 1 400); do grep -q "locomotion node up" $RUN/node.log && break; sleep 1; done
step "node up after ${i} s: $(grep -o 'locomotion node up[^\"]*' $RUN/node.log | head -1 | cut -c1-160)"
sleep 3
cap() { timeout 20 ros2 node info /spiderx_locomotion > $RUN/captures/$1_node_info.txt 2>&1
  timeout 20 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $RUN/captures/$1_action_info.txt 2>&1
  for t in /leg_trajectory_controller/joint_trajectory /cmd_vel /spiderx/locomotion/status; do echo "== $t"; timeout 20 ros2 topic info -v $t; done > $RUN/captures/$1_topics.txt 2>&1; }
step "captures before"; cap before
setsid ros2 bag record -s sqlite3 --include-hidden-topics -o $BAG /spiderx/locomotion/status /joint_states \
  /spiderx/sim/world_poses /cmd_vel /leg_trajectory_controller/joint_trajectory /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/follow_joint_trajectory/_action/feedback > $RUN/bag_record.log 2>&1 &
# NOT _action/status: the locomotion node counts subscribers of the action's status topic as
# action clients (foreign_action_clients) and refuses arm (command_owner_conflict). Attempt 1
# of this rehearsal recorded it and was refused, as designed.
BP=$!; sleep 4
# not /clock: about 575 Hz here (73k messages in 127 s in attempt 3); sim time is in the
# joint-state stamps and the status
( sleep 120; top -b -n1 -w 160 | head -22 > $RUN/captures/top_during.txt;
  ps -o pid,ni,pcpu,args -C ruby,python3,parameter_bridge,robot_state_publisher -w | cut -c1-160 >> $RUN/captures/top_during.txt ) &
step "driver"; python3 $EVT/s2_driver.py --out $RUN/driver --levels 0,1,2 > $RUN/driver.out 2>&1; DRC=$?; step "driver exit $DRC: $(tail -1 $RUN/driver.out | cut -c1-200)"
sleep 3; step "captures after"; cap after
kill -INT -- -$BP 2>/dev/null; for i in $(seq 1 30); do pgrep -g $BP >/dev/null || break; sleep 0.5; done
NP=$(cat $RUN/node.pid); kill -INT -- -$NP 2>/dev/null; for i in $(seq 1 30); do pgrep -g $NP >/dev/null || break; sleep 0.5; done
pgrep -g $NP >/dev/null && { kill -KILL -- -$NP; step "node group SIGKILL"; }
step "node stopped"; pkill -f m55_teleop_keyboard 2>/dev/null
# Phase C change 3: screenshot 2, only after a completed script (no goal in flight: the node is
# stopped) and only if the simulator is still up; it never delays cleanup by more than 15 s.
if [ "$DRC" = 0 ] && pgrep -g "$(cat $RUN/launch.pid)" > /dev/null; then
  env -i PATH=/usr/bin:/bin DISPLAY=:77 timeout 15 /usr/bin/import -display :77 -window root $RUN/captures/screenshot2_after_script.png > $RUN/captures/screenshot2.log 2>&1
  step "screenshot 2: exit $?"
else
  step "screenshot 2 skipped (driver exit $DRC or launch gone)"
fi
step "stop launch"; bash $EVT/stage_stop.sh $RUN > /dev/null
step end
