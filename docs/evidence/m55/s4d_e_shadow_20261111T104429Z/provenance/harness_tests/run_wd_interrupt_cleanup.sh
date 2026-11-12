#!/usr/bin/env bash
T=$(dirname "$(realpath "$0")"); R=$(cat $T/root.txt)
for c in W1 W2 W3; do mkdir -p $T/$c/tools $T/$c/run; cp -p $R/tools/watchdog.sh $T/$c/tools/
  printf '#!/usr/bin/env bash\necho "STUB cleanup.sh $1 $(date -u +%%T.%%N)"\n' > $T/$c/tools/cleanup.sh; printf '#!/usr/bin/env bash\necho "STUB restore_cmd.sh $(date -u +%%T.%%N)"\n' > $T/$c/tools/restore_cmd.sh; done
t0=$(date +%s); bash $T/W1/tools/watchdog.sh $T/W1/run 6 & W=$!; wait $W; WRC=$?
echo "W1 (6 s): exit $WRC after $(( $(date +%s) - t0 )) s; stopped.flag $([ -f $T/W1/run/stopped.flag ] && echo yes); actions: $(grep -o 'STUB [a-z_]*\.sh' $T/W1/run/watchdog.txt | tr '\n' ' ')"
t0=$(date +%s); bash $T/W2/tools/watchdog.sh $T/W2/run 60 & W=$!; sleep 2; touch $T/W2/run/stopped.flag; wait $W; WRC=$?
echo "W2 (60 s, stopped.flag at 2 s): exit $WRC after $(( $(date +%s) - t0 )) s; acted: $([ -f $T/W2/run/watchdog.txt ] && echo yes || echo no)"
t0=$(date +%s); bash $T/W3/tools/watchdog.sh $T/W3/run & W=$!; sleep 1; E=$(cat $T/W3/run/watchdog_deadline_epoch.txt); touch $T/W3/run/stopped.flag; wait $W; WRC=$?
echo "W3 (default): deadline - start = $(( E - t0 )) s; exit $WRC; acted: $([ -f $T/W3/run/watchdog.txt ] && echo yes || echo no)"
for c in I1 I2; do bash $T/mk.sh $c disabled
  C=$T/$c; mkdir -p $C/bagparent; bash $C/tools/watchdog.sh $C/run 7200 & WP=$!; sleep 1
  setsid bash -c "bash $C/tools/s4d_e_shadow_guarded.sh $C/run $C/bagparent/bag; echo \"guarded exit \$?\"" > $C/wrapper.out 2>&1 & PG=$!
  for i in $(seq 1 60); do grep -q "pre-key checks: waiting" $C/run/steps.log 2>/dev/null && break; sleep 1; done
  if [ $c = I2 ]; then echo "test go" > $C/run/go.flag; for i in $(seq 1 60); do grep -q started $C/run/driver/stub_driver.log 2>/dev/null && break; sleep 0.5; done
    echo "I2 before the interruption: driver $(cat $C/run/driver.pid) running: $(kill -0 $(cat $C/run/driver.pid) 2>/dev/null && echo yes); cmdline: $(tr '\0' ' ' < /proc/$(cat $C/run/driver.pid)/cmdline | cut -c1-90); recorder group $(cat $C/run/bag.pid): $(pgrep -g $(cat $C/run/bag.pid) | wc -l) process(es)"; fi
  LP=$(cat $C/run/launch.pid); XP=$(cat $C/run/xvfb.pid); BP=$(cat $C/run/bag.pid 2>/dev/null)
  kill -TERM -- -$PG; for i in $(seq 1 90); do kill -0 $PG 2>/dev/null || pgrep -g $PG > /dev/null || break; sleep 1; done
  echo "$c: wrapper group $PG gone: $(pgrep -g $PG > /dev/null && echo NO || echo yes); $(tail -1 $C/wrapper.out); cleanup runs: $(grep -c 'cleanup start' $C/run/cleanup.log)"
  echo "   launch session $LP: $(pgrep -s $LP | wc -l) left; xvfb $XP alive: $(kill -0 $XP 2>/dev/null && echo YES || echo no); recorder group ${BP:-none}: $([ -n "$BP" ] && pgrep -g $BP | wc -l || echo n/a) left; driver.pid: $(cat $C/run/driver.pid 2>/dev/null || echo none); stub driver log: $(tr '\n' '|' < $C/run/driver/stub_driver.log 2>/dev/null)"
  echo "   cleanup.log: $(grep -E '^driver|recorder|launch|Xvfb' $C/run/cleanup.log | tr '\n' '|' | cut -c1-260)"
  touch $C/run/stopped.flag; wait $WP; done
# C2: owned-process cleanup with the E driver's command line; an unowned decoy survives
C=$T/C2; mkdir -p $C/tools $C/run/driver; cp -p $R/tools/cleanup.sh $C/tools/
printf '#!/usr/bin/env bash\nRUN=$1; LP=$(cat $RUN/launch.pid); kill -INT -- -$LP 2>/dev/null; sleep 1; echo "stub stage_stop: SIGINT to group $LP" >> $RUN/shutdown.txt\n' > $C/tools/stage_stop.sh
cd $C/run
bash -c 'echo $$ > driver.pid; exec -a "python3 /x/scripts/s4/s4d_e_driver.py --out d --shadow-rehearsal" sleep 600' &
python3 -c '
import os, pty, sys, time
pid, fd = pty.fork()
if pid == 0:
    os.execvp("bash", ["bash", "-c", "(exec -a \"m55_teleop_keyboard child-of-teleop-session\" sleep 600) & exec -a \"ros2 run spiderx_controller m55_teleop_keyboard\" sleep 600"])
open("teleop.pid", "w").write(str(pid)); time.sleep(600)' &
setsid bash -c 'exec -a "python3 ros2 bag record -s sqlite3 dummy" sleep 600' & echo $! > bag.pid
setsid bash -c 'set -m; (trap "" INT; exec -a "gz sim server dummy-ignores-SIGINT" sleep 600) & (exec -a "ign gazebo dummy" sleep 600) & exec -a "ros2 launch dummy" sleep 600' & echo $! > launch.pid
bash -c 'exec -a "Xvfb :99 dummy" sleep 600' & echo $! > xvfb.pid
bash -c 'exec -a "python3 /x/scripts/s4/s4d_e_driver.py DECOY-unowned" sleep 600' & echo $! > ../decoy.pid
sleep 1.5; TP=$(cat teleop.pid); echo "{\"mono\": 1.0, \"event\": \"teleop_started\", \"pid\": $TP}" > driver/driver.jsonl
echo "C2 == cleanup.sh (E driver name)"; bash $C/tools/cleanup.sh $C/run | sed 's/^/   /'
for p in $(cat driver.pid) $(cat xvfb.pid); do kill -0 $p 2>/dev/null && echo "   ALIVE owned $p" || echo "   gone $p"; done
echo "   teleop session $TP members: $(pgrep -s $TP | wc -l); launch session $(cat launch.pid) members: $(pgrep -s $(cat launch.pid) | wc -l); recorder group $(cat bag.pid) members: $(pgrep -g $(cat bag.pid) | wc -l)"
kill -0 $(cat ../decoy.pid) 2>/dev/null && echo "   decoy $(cat ../decoy.pid) (same driver name, not recorded) still alive: correctly untouched" || echo "   DECOY KILLED (wrong)"
kill $(cat ../decoy.pid) 2>/dev/null; pkill -f "child-of-teleop-session" 2>/dev/null; true
