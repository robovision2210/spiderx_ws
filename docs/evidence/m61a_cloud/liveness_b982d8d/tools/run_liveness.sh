#!/usr/bin/env bash
# One supervised NO-MOTION launch for pose/clock liveness evidence (not a criterion-6 run):
# launch (frozen stage scripts), controllers active + 3 s, then three phases - A running,
# B paused (gz WorldControl pause: true), C resumed - each with gz-side captures of pose/info,
# /stats and the clock, the read-only ROS stream probe and the observer --preflight; then the
# Ctrl+C-equivalent stop. Nothing is commanded; no goal; the world pause is a simulator control.
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
step() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a $RUN/steps.log; }
W=spiderx_fortress
phase() {   # $1 = label
  local P=$1 L=$RUN/liveness
  timeout 20 ign topic -e -n 3 -t /world/$W/pose/info > $L/${P}_gz_pose_info_first3.txt 2>&1
  sleep 1
  timeout 20 ign topic -e -n 3 -t /world/$W/pose/info > $L/${P}_gz_pose_info_1s_later.txt 2>&1
  timeout 20 ign topic -e -n 2 -t /stats > $L/${P}_gz_stats.txt 2>&1
  timeout 20 ign topic -e -n 2 -t /world/$W/clock > $L/${P}_gz_world_clock.txt 2>&1
  python3 $EVT/probe_streams.py --domain-id 0 --seconds 5 --out $L/${P}_ros_probe.json \
    > $L/${P}_ros_probe.txt 2>&1
  ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --preflight \
    --out $L/observer_${P} > $L/${P}_preflight.txt 2>&1
  echo "preflight exit=$?" >> $L/${P}_preflight.txt
}
step "start"; bash $EVT/stage_start.sh $RUN
step "wait controllers"; bash $EVT/stage_wait_controllers.sh $RUN || { step "CONTROLLERS NOT ACTIVE"; bash $EVT/stage_stop.sh $RUN; exit 1; }
source $EVT/env.sh
mkdir -p $RUN/liveness
timeout 20 ign topic -i -t /world/$W/pose/info > $RUN/liveness/gz_pose_info_publishers.txt 2>&1
timeout 30 ign model -m spiderx > $RUN/liveness/ign_model_all.txt 2>&1
step "A running"; phase A
step "pause"
timeout 10 ign service -s /world/$W/control --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean --timeout 5000 --req 'pause: true' > $RUN/liveness/B_pause_reply.txt 2>&1
sleep 2
step "B paused"; phase B
step "resume"
timeout 10 ign service -s /world/$W/control --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean --timeout 5000 --req 'pause: false' > $RUN/liveness/C_resume_reply.txt 2>&1
sleep 3
step "C resumed"; phase C
step "stop"; bash $EVT/stage_stop.sh $RUN
step "end"
