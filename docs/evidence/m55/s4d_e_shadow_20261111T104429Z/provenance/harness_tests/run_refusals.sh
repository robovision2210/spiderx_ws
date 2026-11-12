#!/usr/bin/env bash
# Non-interactive, like the harness. Each case: the E shadow run script (test copy) until it refuses or aborts.
T=$(dirname "$(realpath "$0")")
run() { C=$T/$1; mkdir -p $C/bagparent; bash $C/tools/s4d_e_shadow_run.sh $C/run $C/bagparent/bag > $C/out.txt 2>&1; echo "$1 exit $?: $(grep -E 'REFUSED|ABORT' $C/out.txt | tail -1 | cut -c12-230)"; }
bash $T/mk.sh S1 missing;  run S1
bash $T/mk.sh S2 enabled;  run S2
bash $T/mk.sh S3 disabled; run S3
bash $T/mk.sh S4 disabled; bash $T/S4/tools/watchdog.sh $T/S4/run 2000 & sleep 1; run S4; touch $T/S4/run/stopped.flag
bash $T/mk.sh S5 disabled; bash $T/S5/tools/watchdog.sh $T/S5/run 12000 & sleep 1; run S5; touch $T/S5/run/stopped.flag
# S6: OD8 before the first key (deadline moved to now + 1000 s during the pre-key wait).
# S7: OD16 before recording (deadline moved to now + 11000 s: OD8 passes, the disk rule fails).
for c in S6 S7; do bash $T/mk.sh $c disabled; bash $T/$c/tools/watchdog.sh $T/$c/run 7200 & sleep 1
  ( C=$T/$c; for i in $(seq 1 60); do grep -q "pre-key checks: waiting" $C/run/steps.log 2>/dev/null && break; sleep 1; done
    [ $c = S6 ] && echo $(( $(date +%s) + 1000 )) > $C/run/watchdog_deadline_epoch.txt
    [ $c = S7 ] && echo $(( $(date +%s) + 11000 )) > $C/run/watchdog_deadline_epoch.txt
    echo "test: pre-key checks PASS (stub)" > $C/run/go.flag ) &
  run $c; touch $T/$c/run/stopped.flag; done
wait
for c in S6 S7; do echo "$c: key/driver started: $([ -f $T/$c/run/driver.pid ] && echo YES || echo no); recorder started: $([ -f $T/$c/run/bag.pid ] && echo YES || echo no)"; done
