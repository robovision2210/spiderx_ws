#!/usr/bin/env bash
# Plan section 4 E (start): the bag, the read-only observer (90 s), the confirmation FIFO and the
# live tool, each in its own session with a pid file. The confirmation word is written
# separately, only after live.txt shows readiness #1 passed and it has been inspected.
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
cd $RUN
setsid ros2 bag record -s sqlite3 -o $RUN/bag /spiderx/sim/world_poses /joint_states /clock \
  /robot_description /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/joint_trajectory > $RUN/bag_record.log 2>&1 &
echo $! > $RUN/bag.pid
sleep 3
setsid ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 90 \
  --out $RUN/observer > $RUN/observe_during.txt 2>&1 &
echo $! > $RUN/observer.pid
mkfifo $RUN/confirm
setsid sleep 3600 > $RUN/confirm &           # keeps the write end open: no EOF before the word
echo $! > $RUN/confirm_holder.pid
date -u +%FT%T.%NZ > $RUN/tool_start_utc.txt
setsid bash -c "PYTHONUNBUFFERED=1 ros2 run spiderx_controller m6_gait_replay.py --live \
  --domain-id 0 --out $RUN/m61_run < $RUN/confirm > $RUN/live.txt 2>&1; \
  echo \"exit=\$?\" >> $RUN/live.txt; date -u +%FT%T.%NZ > $RUN/tool_end_utc.txt" &
echo $! > $RUN/tool.pid
echo "bag $(cat bag.pid) observer $(cat observer.pid) holder $(cat confirm_holder.pid) tool $(cat tool.pid)"
