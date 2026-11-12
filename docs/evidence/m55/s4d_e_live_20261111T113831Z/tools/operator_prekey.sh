#!/usr/bin/env bash
# S4d-E LIVE operator pre-key procedure (run-local; new for E: the S4d-M live operator's pre-key steps, scripted
# so that the checkpoint cannot be missed as it was in the E shadow run). Started by the operator right after
# the guarded run; it waits for the PRE-KEY CHECKPOINT itself, detected by EITHER the run script's
# RUN/premotion_wait.flag OR its fixed "PRE-KEY CHECKPOINT" steps.log line (both recorded), then:
#  1. premotion_checks.sh (the LIVE DISPATCH checks; unchanged criteria);
#  2. settle_check.py --dispatch (unchanged criteria: >= 10 s of sim of fresh samples, DISPATCH/gate True/DISARMED);
#  3. startup lines: no 'process has died' (ERROR/WARN lines recorded; not a criterion, as in S4d-M live);
#  4. one screenshot at rest (recorded; not a criterion);
#  5. go.flag only if 1-3 PASS; otherwise abort.flag with the reasons (the run stops with no key pressed).
# It presses no key, sends nothing and changes no parameter. Exit 0 go, 1 abort, 2 the run ended first.
# Usage: operator_prekey.sh <run dir> [wait s]
RUN=$1; EVT=$(dirname "$RUN")/tools; WAIT_S=${2:-1500}; D=$RUN/premotion
log() { echo "[$(date -u +%T)] operator: $*" | tee -a $RUN/operator_prekey.log; }
log "waiting for the PRE-KEY CHECKPOINT (premotion_wait.flag or the steps.log label), at most $WAIT_S s"
for i in $(seq 1 $WAIT_S); do
  FL=no; LB=no; [ -f $RUN/premotion_wait.flag ] && FL=yes; grep -q "PRE-KEY CHECKPOINT" $RUN/steps.log 2>/dev/null && LB=yes
  { [ $FL = yes ] || [ $LB = yes ]; } && break
  if grep -q -E "REFUSED|ABORT|\] end$" $RUN/steps.log 2>/dev/null; then log "the run ended before the checkpoint: $(tail -1 $RUN/steps.log)"; exit 2; fi
  sleep 1
done
[ $FL = yes ] || [ $LB = yes ] || { log "no checkpoint within $WAIT_S s: abort.flag"; echo "operator: no checkpoint within $WAIT_S s" > $RUN/abort.flag; exit 1; }
sleep 1; [ -f $RUN/premotion_wait.flag ] && FL=yes; grep -q "PRE-KEY CHECKPOINT" $RUN/steps.log 2>/dev/null && LB=yes
log "checkpoint seen after ${i} s: premotion_wait.flag $FL, steps.log label $LB"
mkdir -p $D $RUN/captures; WHY=""
bash $EVT/premotion_checks.sh $RUN > $RUN/premotion_stdout.txt 2>&1
grep -q "^PREMOTION VERDICT: PASS$" $D/verdict.txt 2>/dev/null || WHY="$WHY premotion($(cat $D/verdict.txt 2>/dev/null | head -c 200))"
log "premotion: $(cat $D/verdict.txt 2>/dev/null)"
( source $EVT/env.sh; cd / && PYTHONDONTWRITEBYTECODE=1 python3 $EVT/settle_check.py --out $D/settle_check.json --dispatch --timeout-s 600 ) > $D/settle_check.log 2>&1
grep -q "^SETTLE VERDICT: PASS$" $D/settle_check.log || WHY="$WHY settle($(tail -1 $D/settle_check.log | head -c 200))"
log "settle: $(tail -1 $D/settle_check.log)"
NE=$(grep -c -E "\[(ERROR|FATAL)\]" $RUN/launch.log); ND=$(grep -c "process has died" $RUN/launch.log)
{ echo "[ERROR]/[FATAL] lines: $NE; 'process has died': $ND"; grep -n -E "\[(ERROR|FATAL)\]" $RUN/launch.log | sed 's/\x1b\[[0-9;]*m//g'
  echo "== [WARN] lines (recorded, not a criterion)"; grep -n "\[WARN\]" $RUN/launch.log | sed 's/\x1b\[[0-9;]*m//g'; } > $D/startup_lines.txt
[ "$ND" = 0 ] || WHY="$WHY process_died($ND)"
log "startup lines: [ERROR]/[FATAL] $NE (recorded), process has died $ND"
env -i PATH=/usr/bin:/bin DISPLAY=:77 timeout 15 /usr/bin/import -display :77 -window root $RUN/captures/screenshot1_rest_before_first_key.png > $RUN/captures/screenshot1.log 2>&1
SRC=$?; date -u +%FT%TZ > $RUN/captures/screenshot1_utc.txt; log "screenshot 1: exit $SRC (recorded, not a criterion)"
if [ -z "$WHY" ]; then
  echo "pre-key checks PASS ($(grep -o 'PASS: one process maps the plugin.*' $D/candidates.txt | head -1 | cut -c1-120); $(grep -c 'PASS' $D/library_checks.txt) library lines PASS; gain/update_rate/period per log_checks and params; $(grep -o 'node action clients:.*' $D/ownership.txt | head -1); settle PASS $(python3 -c "import json; d=json.load(open('$D/settle_check.json')); w=d['window']; print(f\"{w['sim_to_s']-w['sim_from_s']:.2f} s sim, {d['fresh_samples']} fresh samples, z range {w['z_range_m']*1e6:.1f} um, tilt {w['max_tilt_rad']:.1e} rad\")" 2>/dev/null); startup: process has died 0, [ERROR]/[FATAL] $NE recorded)" > $RUN/go.flag
  log "GO: $(cat $RUN/go.flag | cut -c1-300)"; exit 0
fi
echo "operator pre-key checks FAILED:$WHY" > $RUN/abort.flag; log "ABORT:$WHY"; exit 1
