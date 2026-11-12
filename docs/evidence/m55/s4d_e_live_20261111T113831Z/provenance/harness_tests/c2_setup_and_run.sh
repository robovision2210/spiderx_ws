#!/usr/bin/env bash
# Non-interactive (no job control), like the real harness. Owned processes: the E LIVE driver command line (no
# --shadow-rehearsal), its teleop pty session, the recorder group, the launch session (a SIGINT-ignoring member
# included) and Xvfb. Two UNOWNED decoys: one with the same E driver command line, one teleop-named.
T=$1; cd $T/run
bash -c 'echo $$ > driver.pid; exec -a "python3 /x/scripts/s4/s4d_e_driver.py --out d" sleep 600' &
python3 -c '
import os, pty, sys, time
pid, fd = pty.fork()
if pid == 0:
    os.execvp("bash", ["bash", "-c", "(exec -a \"m55_teleop_keyboard child-of-teleop-session\" sleep 600) & exec -a \"ros2 run spiderx_controller m55_teleop_keyboard\" sleep 600"])
open("teleop.pid", "w").write(str(pid)); time.sleep(600)' &
setsid bash -c 'exec -a "python3 ros2 bag record -s sqlite3 dummy" sleep 600' & echo $! > bag.pid
setsid bash -c 'set -m; (trap "" INT; exec -a "gz sim server dummy-ignores-SIGINT" sleep 600) & (exec -a "ign gazebo dummy" sleep 600) & exec -a "ros2 launch dummy" sleep 600' & echo $! > launch.pid
bash -c 'exec -a "Xvfb :99 dummy" sleep 600' & echo $! > xvfb.pid
bash -c 'exec -a "python3 /x/scripts/s4/s4d_e_driver.py --out d" sleep 600' & echo $! > ../decoy_driver.pid
bash -c 'exec -a "m55_teleop_keyboard DECOY-unowned" sleep 600' & echo $! > ../decoy_teleop.pid
sleep 1.5; TP=$(cat teleop.pid); echo "{\"mono\": 1.0, \"event\": \"teleop_started\", \"pid\": $TP}" > driver/driver.jsonl
echo "== before cleanup: recorded driver $(cat driver.pid), teleop session $TP, recorder group $(cat bag.pid), launch $(cat launch.pid), xvfb $(cat xvfb.pid); decoys $(cat ../decoy_driver.pid) $(cat ../decoy_teleop.pid) NOT recorded"
echo "== cleanup.sh"; bash $T/tools/cleanup.sh $T/run
echo "== after"; for p in $(cat driver.pid) $(cat xvfb.pid); do kill -0 $p 2>/dev/null && echo "ALIVE owned $p" || echo "gone $p"; done
echo "teleop session $TP members: $(pgrep -s $TP | wc -l); launch session $(cat launch.pid) members: $(pgrep -s $(cat launch.pid) | wc -l); recorder group $(cat bag.pid) members: $(pgrep -g $(cat bag.pid) | wc -l)"
for d in decoy_driver decoy_teleop; do kill -0 $(cat ../$d.pid) 2>/dev/null && echo "$d $(cat ../$d.pid) still alive (not owned: correctly untouched)" || echo "$d KILLED (wrong)"; done
kill -KILL $(cat ../decoy_driver.pid) $(cat ../decoy_teleop.pid) 2>/dev/null; echo "(test teardown: the two decoys stopped by the test itself)"
echo "-- stub shutdown.txt"; cat shutdown.txt
