#!/usr/bin/env bash
# Non-interactive, like the harness. Each case: the real live run script (test copy) until it refuses or aborts.
T=$(dirname "$(realpath "$0")"); G=$T/repo
run() { C=$T/$1; mkdir -p $C/bagparent; bash $C/tools/s4d_live_run.sh $C/run $C/bagparent/bag > $C/out.txt 2>&1; echo "$1 exit $?: $(grep -E 'REFUSED|ABORT' $C/out.txt | tail -1 | cut -c12-220)"; }
bash $T/mk.sh L1 real;     run L1
bash $T/mk.sh L2 disabled; git -C $G switch -q local/m55-s4d-m-TEST; echo local/m55-s4d-m-TEST > $T/L2/enabling_branch.txt; run L2
bash $T/mk.sh L3 enabled;  run L3
bash $T/mk.sh L4 enabled;  git -C $G switch -q master; echo local/m55-s4d-m-TEST > $T/L4/enabling_branch.txt; run L4
bash $T/mk.sh L4w enabled; git -C $G switch -q local/m55-s4d-m-WIDE; echo local/m55-s4d-m-WIDE > $T/L4w/enabling_branch.txt; run L4w
git -C $G switch -q local/m55-s4d-m-TEST
bash $T/mk.sh L5 enabled;  echo local/m55-s4d-m-TEST > $T/L5/enabling_branch.txt; run L5
bash $T/mk.sh L6 enabled;  echo local/m55-s4d-m-TEST > $T/L6/enabling_branch.txt; bash $T/L6/tools/watchdog.sh $T/L6/run 6000 & W6=$!; sleep 1; run L6; touch $T/L6/run/stopped.flag
bash $T/mk.sh L7 enabled;  echo local/m55-s4d-m-TEST > $T/L7/enabling_branch.txt; bash $T/L7/tools/watchdog.sh $T/L7/run 12000 & W7=$!; sleep 1; run L7; touch $T/L7/run/stopped.flag
# L8: OD8 before the first key. L9: OD16 before recording (deadline moved to now + 11000 s: OD8 passes, the disk rule fails)
for c in L8 L9; do bash $T/mk.sh $c enabled; echo local/m55-s4d-m-TEST > $T/$c/enabling_branch.txt; bash $T/$c/tools/watchdog.sh $T/$c/run 7200 & sleep 1
  ( C=$T/$c; for i in $(seq 1 60); do grep -q "pre-key checks: waiting" $C/run/steps.log 2>/dev/null && break; sleep 1; done
    [ $c = L8 ] && echo $(( $(date +%s) + 1000 )) > $C/run/watchdog_deadline_epoch.txt
    [ $c = L9 ] && echo $(( $(date +%s) + 11000 )) > $C/run/watchdog_deadline_epoch.txt
    echo "test: pre-key checks PASS (stub)" > $C/run/go.flag ) &
  run $c; touch $T/$c/run/stopped.flag; done
wait
