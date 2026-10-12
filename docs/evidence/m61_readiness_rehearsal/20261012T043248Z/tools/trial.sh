#!/usr/bin/env bash
# One read-only readiness trial: bag, then the observer, BEFORE the probe; then the probe.
#   trial.sh <run dir> <index> <new|old> <delay_s>
# delay 0 reproduces the attempt's start-up (bag, 3 s, observer, tool at once); delay 10 is a
# quiet start (the observer has finished discovery). new = the production rehearsal entry point
# (61f3ffc); old = the a1a7d60 readiness composition replayed read-only (old_path_probe.py).
# Neither can send: both use RehearsalTransport. No --live, no gate change.
RUN=$(cd "$1" && pwd); K=$2; MODE=$3; DELAY=$4; EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
T=$RUN/trials/$(printf %02d $K)_${MODE}_d${DELAY}; mkdir -p $T
setsid ros2 bag record -s sqlite3 -o $T/bag /spiderx/sim/world_poses /joint_states /clock \
  /robot_description /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/joint_trajectory > $T/bag_record.log 2>&1 &
BAG=$!; echo $BAG > $T/bag.pid
sleep 3
date -u +%FT%T.%NZ > $T/observer_start_utc.txt
setsid ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 30 \
  --out $T/observer > $T/observe.txt 2>&1 &
OBS=$!; echo $OBS > $T/observer.pid
sleep $DELAY
date -u +%FT%T.%NZ > $T/probe_start_utc.txt
if [ "$MODE" = new ]; then
  PYTHONUNBUFFERED=1 ros2 run spiderx_controller m61_readiness_rehearsal --domain-id 0 \
    --out $T/rehearsal > $T/probe.txt 2>&1
else
  PYTHONUNBUFFERED=1 python3 $EVT/old_path_probe.py --domain-id 0 --out $T/old_path.json \
    > $T/probe.txt 2>&1
fi
echo "exit=$?" >> $T/probe.txt
date -u +%FT%T.%NZ > $T/probe_end_utc.txt
while kill -0 $OBS 2>/dev/null; do sleep 0.5; done
echo "observe exit recorded above" >> $T/observe.txt
kill -INT -- -$BAG 2>/dev/null
for i in $(seq 1 30); do pgrep -g $BAG > /dev/null || break; sleep 0.5; done
pgrep -g $BAG -a > $T/bag_leftover.txt 2>&1 || echo none > $T/bag_leftover.txt
cp $T/bag/metadata.yaml $T/bag_metadata.yaml 2>/dev/null
echo "trial $K $MODE delay $DELAY: $(grep -h 'old path\|^READY\|^NOT READY\|action discovery' $T/probe.txt | tr '\n' ' ')"
