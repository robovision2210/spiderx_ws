#!/usr/bin/env bash
# Plan A2, one read-only rehearsal: bag and observer first, 10 s, then the rehearsal entry point
# (m61_readiness_rehearsal: no session, no goal, send/cancel raise). Disabled build only.
#   a2_rehearse.sh <run dir> <index>
RUN=$(cd "$1" && pwd); K=$2; EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
T=$RUN/rehearsal_$K; mkdir -p $T
setsid ros2 bag record -s sqlite3 -o $T/bag /spiderx/sim/world_poses /joint_states /clock \
  /robot_description /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/joint_trajectory > $T/bag_record.log 2>&1 &
BAG=$!
sleep 3
setsid ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 45 \
  --out $T/observer > $T/observe.txt 2>&1 &
OBS=$!
sleep 10
date -u +%FT%T.%NZ > $T/probe_start_utc.txt
PYTHONUNBUFFERED=1 ros2 run spiderx_controller m61_readiness_rehearsal --domain-id 0 \
  --out $T/rehearsal > $T/probe.txt 2>&1
echo "exit=$?" >> $T/probe.txt
while kill -0 $OBS 2>/dev/null; do sleep 0.5; done
kill -INT -- -$BAG 2>/dev/null
for i in $(seq 1 30); do pgrep -g $BAG > /dev/null || break; sleep 0.5; done
cp $T/bag/metadata.yaml $T/bag_metadata.yaml 2>/dev/null; rm -rf $T/bag
echo "rehearsal $K: $(grep -h '^READY\|^NOT READY\|action discovery\|exit=' $T/probe.txt | tr '\n' ' ')"
