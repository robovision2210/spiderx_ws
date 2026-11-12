#!/usr/bin/env bash
T=$(dirname "$(realpath "$0")"); R=/root/spiderx_evidence/s4d_e_live_20261111T113831Z
# --- watchdog: fire (stub cleanup, stub restore: order recorded), stand-down, 7200 s default
for c in W1 W2 W3; do mkdir -p $T/$c/tools $T/$c/run; cp -p $R/tools/watchdog.sh $T/$c/tools/
  printf '#!/usr/bin/env bash\necho "STUB cleanup.sh $1 $(date -u +%%T.%%N)"\n' > $T/$c/tools/cleanup.sh; printf '#!/usr/bin/env bash\necho "STUB restore_cmd.sh $(date -u +%%T.%%N)"\n' > $T/$c/tools/restore_cmd.sh; done
t0=$(date +%s); bash $T/W1/tools/watchdog.sh $T/W1/run 6 & W=$!; wait $W; WRC=$?
echo "W1 (6 s): exit $WRC after $(( $(date +%s) - t0 )) s; stopped.flag $([ -f $T/W1/run/stopped.flag ] && echo yes); actions: $(grep -o 'STUB [a-z_]*\.sh' $T/W1/run/watchdog.txt | tr '\n' ' ')"
t0=$(date +%s); bash $T/W2/tools/watchdog.sh $T/W2/run 60 & W=$!; sleep 2; touch $T/W2/run/stopped.flag; wait $W; WRC=$?
echo "W2 (60 s, stopped.flag at 2 s): exit $WRC after $(( $(date +%s) - t0 )) s; acted: $([ -f $T/W2/run/watchdog.txt ] && echo yes || echo no)"
t0=$(date +%s); bash $T/W3/tools/watchdog.sh $T/W3/run & W=$!; sleep 1; E=$(cat $T/W3/run/watchdog_deadline_epoch.txt); touch $T/W3/run/stopped.flag; wait $W; WRC=$?
echo "W3 (default): deadline - start = $(( E - t0 )) s; exit $WRC; acted: $([ -f $T/W3/run/watchdog.txt ] && echo yes || echo no)"
# --- interruption of the guarded wrapper: I1 during the pre-key wait (before the driver), I2 while the (stub) driver runs
for c in I1 I2; do bash $T/mk.sh $c enabled; echo local/m55-s4d-e-TEST > $T/$c/enabling_branch.txt; git -C $T/repo switch -q local/m55-s4d-e-TEST
  C=$T/$c; mkdir -p $C/bagparent; bash $C/tools/watchdog.sh $C/run 7200 & WP=$!; sleep 1
  setsid bash -c "bash $C/tools/s4d_e_live_guarded.sh $C/run $C/bagparent/bag; echo \"guarded exit \$?\"" > $C/wrapper.out 2>&1 & PG=$!
  for i in $(seq 1 60); do [ -f $C/run/premotion_wait.flag ] && grep -q "PRE-KEY CHECKPOINT" $C/run/steps.log 2>/dev/null && break; sleep 1; done
  if [ $c = I2 ]; then echo "test go" > $C/run/go.flag; for i in $(seq 1 60); do grep -q started $C/run/driver/stub_driver.log 2>/dev/null && break; sleep 0.5; done
    echo "I2 before the interruption: driver $(cat $C/run/driver.pid) running: $(kill -0 $(cat $C/run/driver.pid) 2>/dev/null && echo yes); recorder group $(cat $C/run/bag.pid): $(pgrep -g $(cat $C/run/bag.pid) | wc -l) process(es)"; fi
  LP=$(cat $C/run/launch.pid); XP=$(cat $C/run/xvfb.pid); BP=$(cat $C/run/bag.pid 2>/dev/null)
  kill -TERM -- -$PG; for i in $(seq 1 90); do kill -0 $PG 2>/dev/null || pgrep -g $PG > /dev/null || break; sleep 1; done
  echo "$c: wrapper group $PG gone: $(pgrep -g $PG > /dev/null && echo NO || echo yes); $(tail -1 $C/wrapper.out); cleanup runs: $(grep -c 'cleanup start' $C/run/cleanup.log)"
  echo "   launch session $LP: $(pgrep -s $LP | wc -l) left; xvfb $XP alive: $(kill -0 $XP 2>/dev/null && echo YES || echo no); recorder group ${BP:-none}: $([ -n "$BP" ] && pgrep -g $BP | wc -l || echo n/a) left; driver.pid: $(cat $C/run/driver.pid 2>/dev/null || echo none); stub driver log: $(tr '\n' '|' < $C/run/driver/stub_driver.log 2>/dev/null)"
  echo "   steps tail: $(tail -2 $C/run/steps.log | tr '\n' '|' | cut -c1-160)"
  touch $C/run/stopped.flag; wait $WP; done
