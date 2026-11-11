#!/usr/bin/env bash
# Operator step C: arm the watchdog (7200 s) BEFORE enabling, then the ONE enabling + B1 build sequence.
R=/root/spiderx_evidence/s4d_e_live_20261111T113831Z; RUN=$R/run; J=/root/spiderx_evidence/s4d_e_live_journal.md
[ -e $RUN ] && { echo "run dir exists: STOP"; exit 9; }
mkdir -p $RUN
setsid nohup bash $R/tools/watchdog.sh $RUN 7200 > $R/watchdog.out 2>&1 < /dev/null &
for i in $(seq 1 20); do [ -s $RUN/watchdog.pid ] && [ -s $RUN/watchdog_deadline_epoch.txt ] && break; sleep 0.25; done
WP=$(cat $RUN/watchdog.pid); DL=$(cat $RUN/watchdog_deadline_epoch.txt)
cp $RUN/watchdog_armed.txt $R/provenance/watchdog_armed.txt
echo "- $(date -u +%FT%TZ) watchdog armed (7200 s): pid $WP, deadline $(date -u -d @$DL +%FT%TZ); OD8 (>= 1800 + 900 s left) launch-by $(date -u -d @$((DL-2700)) +%FT%TZ); enabling-to-launch bound 900 s from the enabling start" >> $J
U=$(date -u +%Y%m%dT%H%M%SZ)
echo "- $(date -u +%FT%TZ) enabling + B1 build started: enable_build.sh $U (local/m55-s4d-e-$U; never pushed)" >> $J
bash $R/tools/enable_build.sh $U > $R/enable_build.out 2>&1; RC=$?
echo "enable_build exit $RC"; tail -12 $R/enable_build.out
exit $RC
