#!/usr/bin/env bash
# Plan A2 driver (as at 24782f6, plus the tracking-path probe of the d748891 plan): launch the
# DISABLED build, controllers, preflight, 3 read-only rehearsals (bag and observer first), the
# read-only tracking-path probe once, captures, stop.
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools; WS=/home/user/spiderx_ws
step() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a $RUN/steps.log; }
step "start"; bash $EVT/stage_start.sh $RUN
step "wait controllers"; bash $EVT/stage_wait_controllers.sh $RUN || { step "CONTROLLERS NOT ACTIVE"; bash $EVT/stage_stop.sh $RUN; exit 1; }
step "preflight"; bash $EVT/stage_observe.sh $RUN preflight
for k in 1 2 3; do step "rehearsal $k"; bash $EVT/a2_rehearse.sh $RUN $k | tee -a $RUN/rehearsals.log; sleep 3; done
step "tracking-path probe"; (source $EVT/env.sh; cd $WS
  PYTHONUNBUFFERED=1 python3 docs/evidence/m61_tracking_fix/20261012T081545Z/tools/tracking_path_probe.py \
    --domain-id 0 --duration 40 --out $RUN/probe > $RUN/probe.txt 2>&1; echo "exit=$?" >> $RUN/probe.txt)
step "probe: $(grep -h '^verdict' $RUN/probe.txt | cut -c1-40) $(grep -h '^exit=' $RUN/probe.txt)"
step "captures"; (source $EVT/env.sh; mkdir -p $RUN/captures
  timeout 20 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $RUN/captures/action_info.txt 2>&1
  timeout 20 ros2 topic info -v /leg_trajectory_controller/joint_trajectory > $RUN/captures/command_topic_info.txt 2>&1)
step "stop"; bash $EVT/stage_stop.sh $RUN
step "end"
