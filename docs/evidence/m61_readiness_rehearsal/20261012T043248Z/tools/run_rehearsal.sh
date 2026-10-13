#!/usr/bin/env bash
# The whole no-motion rehearsal: launch, controllers, preflight, 16 trials, captures, stop.
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
step() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a $RUN/steps.log; }
step "start"; bash $EVT/stage_start.sh $RUN
step "wait controllers"; bash $EVT/stage_wait_controllers.sh $RUN || { step "CONTROLLERS NOT ACTIVE"; bash $EVT/stage_stop.sh $RUN; exit 1; }
step "preflight"; bash $EVT/stage_observe.sh $RUN preflight
k=0
for pair in 1 2 3 4 5 6; do
  k=$((k+1)); step "trial $k old d0"; bash $EVT/trial.sh $RUN $k old 0 | tee -a $RUN/trials.log; sleep 3
  k=$((k+1)); step "trial $k new d0"; bash $EVT/trial.sh $RUN $k new 0 | tee -a $RUN/trials.log; sleep 3
done
for pair in 1 2; do
  k=$((k+1)); step "trial $k old d10"; bash $EVT/trial.sh $RUN $k old 10 | tee -a $RUN/trials.log; sleep 3
  k=$((k+1)); step "trial $k new d10"; bash $EVT/trial.sh $RUN $k new 10 | tee -a $RUN/trials.log; sleep 3
done
step "captures"; (source $EVT/env.sh; mkdir -p $RUN/captures
  timeout 20 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $RUN/captures/action_info.txt 2>&1
  timeout 20 ros2 node list > $RUN/captures/nodes.txt 2>&1
  timeout 20 ros2 topic info -v /leg_trajectory_controller/joint_trajectory > $RUN/captures/command_topic_info.txt 2>&1
  timeout 20 ros2 control list_controllers > $RUN/captures/controllers.txt 2>&1)
step "stop"; bash $EVT/stage_stop.sh $RUN
step "end"
