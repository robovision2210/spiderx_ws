#!/usr/bin/env bash
# Non-interactive (no job control), like the real harness: setsid does not fork, so $! is the
# session/group leader, as stage_start.sh and the run script record it.
T=$1; cd $T/run
bash -c 'echo $$ > driver.pid; exec -a "python3 /x/scripts/s4/s4d_driver.py --out d --shadow-rehearsal" sleep 600' &
python3 -c '
import os, pty, sys, time
pid, fd = pty.fork()        # as s2_driver.spawn: the teleop child gets its own session (setsid)
if pid == 0:
    os.execvp("bash", ["bash", "-c", "(exec -a \"m55_teleop_keyboard child-of-teleop-session\" sleep 600) & exec -a \"ros2 run spiderx_controller m55_teleop_keyboard\" sleep 600"])
open("teleop.pid", "w").write(str(pid)); time.sleep(600)' &
setsid bash -c 'exec -a "python3 ros2 bag record -s sqlite3 dummy" sleep 600' & echo $! > bag.pid
setsid bash -c 'set -m; (trap "" INT; exec -a "gz sim server dummy-ignores-SIGINT" sleep 600) & (exec -a "ign gazebo dummy" sleep 600) & exec -a "ros2 launch dummy" sleep 600' & echo $! > launch.pid
bash -c 'exec -a "Xvfb :99 dummy" sleep 600' & echo $! > xvfb.pid
bash -c 'exec -a "m55_teleop_keyboard DECOY-unowned" sleep 600' & echo $! > ../decoy.pid
sleep 1.5; TP=$(cat teleop.pid); echo "{\"mono\": 1.0, \"event\": \"teleop_started\", \"pid\": $TP}" > driver/driver.jsonl
echo "== before cleanup: recorded driver $(cat driver.pid), teleop session $TP, recorder group $(cat bag.pid), launch $(cat launch.pid), xvfb $(cat xvfb.pid); decoy $(cat ../decoy.pid) is NOT recorded"
ps -o pid,pgid,sid,args -p $(cat driver.pid) -p $(cat xvfb.pid) -p $(cat ../decoy.pid); echo "-- session $TP"; ps -o pid,pgid,sid,args -s $TP; echo "-- session $(cat launch.pid)"; ps -o pid,pgid,sid,args -s $(cat launch.pid); echo "-- group $(cat bag.pid)"; ps -o pid,pgid,sid,args -g $(cat bag.pid)
echo "== cleanup.sh"; bash $T/tools/cleanup.sh $T/run
echo "== after"; for p in $(cat driver.pid) $(cat xvfb.pid); do kill -0 $p 2>/dev/null && echo "ALIVE owned $p" || echo "gone $p"; done
echo "teleop session $TP members: $(pgrep -s $TP | wc -l); launch session $(cat launch.pid) members: $(pgrep -s $(cat launch.pid) | wc -l); recorder group $(cat bag.pid) members: $(pgrep -g $(cat bag.pid) | wc -l)"
kill -0 $(cat ../decoy.pid) 2>/dev/null && echo "decoy $(cat ../decoy.pid) still alive (not owned: correctly untouched)" || echo "DECOY KILLED (wrong)"
echo "-- stub shutdown.txt"; cat shutdown.txt
