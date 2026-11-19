#!/usr/bin/env bash
# S4e harness template: ONE session (docs/S4E_CONTRACT.md sections 3-6). NOT AUTHORIZED TO RUN.
# Usage: s4e_session.sh <session dir> <bag dir>. RUN = <session>/run; the operator's observer
# writes <session>/observer and the preflight <session>/preflight.json (the scorer's layout).
#  1. Refusals, nothing launched: mode, installation, gates, ref (checks.sh), the watchdog's time
#     (>= 20 + 15 min) and the disk rule.
#  2. ONE launch (its own session), then the node's "locomotion node up".
#  3. PRE-KEY CHECKPOINT: the operator's checks; go.flag continues, abort.flag (or no decision)
#     stops with no key pressed.
#  4. Time and disk again; captures before; the driver scan; ONE recorder (its own group).
#  5. The operator's session: the observer in the operator's own terminal (never started here);
#     this waits for its summary or abort.flag.
#  6. The driver scan, captures after, recorder and launch stopped (owned only).
SESSION=$1; BAG=$2; RUN=$SESSION/run; H=$(dirname "$(realpath "$0")")
source "$H/env.sh"; source "$H/checks.sh"
mkdir -p "$RUN/node" "$RUN/captures"
step() { echo "[$(date -u +%T)] $*" | tee -a "$RUN/steps.log"; }
refuse() { step "REFUSED before the launch (nothing launched): $*"; step end; exit 1; }
stop_owned() { bash "$H/cleanup.sh" "$RUN" >> "$RUN/cleanup.log" 2>&1; }
abort() { step "ABORT: $*"; stop_owned; step end; exit 1; }
# <pid file> <log> <command>: in its own session, with SIGINT and SIGQUIT at their default
# dispositions (a background job of a non-interactive shell inherits them IGNORED, and a program
# cannot trap a signal ignored on entry), so that the cleanup's SIGINT reaches it.
start_owned() {
  setsid python3 -c 'import os, signal, sys
for s in (signal.SIGINT, signal.SIGQUIT):
    signal.signal(s, signal.SIG_DFL)
os.execvp("bash", ["bash", "-c", sys.argv[1]])' "exec $3" > "$2" 2>&1 &
  echo $! > "$RUN/$1"
}
for c in check_mode check_install check_gates check_ref; do OUT=$($c) || refuse "$OUT"; done
step "inputs: $S4E_MODE, installation $RUNINST, gates, ref: PASS"
OUT=$(wd_check "before the launch") || refuse "$OUT"; step "$OUT"
OUT=$(disk_check "before the launch") || refuse "$OUT"; step "$OUT"
step "launch"
start_owned launch.pid "$RUN/launch.log" "$LAUNCH_CMD"
for i in $(seq 1 "$NODE_UP_WAIT_S"); do grep -q "locomotion node up" "$RUN/launch.log" && break; sleep 1; done
grep -q "locomotion node up" "$RUN/launch.log" || abort "locomotion node not up"
step "PRE-KEY CHECKPOINT: waiting for go.flag or abort.flag (at most $PREMOTION_WAIT_S s)"
for i in $(seq 1 "$PREMOTION_WAIT_S"); do { [ -f "$RUN/go.flag" ] || [ -f "$RUN/abort.flag" ]; } && break; sleep 1; done
[ -f "$RUN/abort.flag" ] && abort "pre-key checks failed: $(head -c 300 "$RUN/abort.flag")"
[ -f "$RUN/go.flag" ] || abort "no pre-key decision within $PREMOTION_WAIT_S s"
OUT=$(wd_check "before the session") || abort "$OUT"; step "$OUT"
OUT=$(disk_check "before recording") || abort "$OUT"; step "$OUT"
bash -c "$CAPTURE_CMD" > "$RUN/captures/before_topics.txt" 2>&1
bash "$H/scan.sh" "$RUN" start | tee -a "$RUN/steps.log"
start_owned bag.pid "$RUN/bag_record.log" "$BAG_CMD"
step "SESSION: in your own terminal: python3 $WS/scripts/s4/s4e_observer.py --out $SESSION/observer --mode $S4E_MODE"
for i in $(seq 1 "$SESSION_WAIT_S"); do { [ -f "$SESSION/observer/observer_summary.json" ] || [ -f "$RUN/abort.flag" ]; } && break; sleep 1; done
[ -f "$SESSION/observer/observer_summary.json" ] && step "observer: $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["result"])' "$SESSION/observer/observer_summary.json")" || step "no observer summary (abort.flag or the session wait)"
bash "$H/scan.sh" "$RUN" end | tee -a "$RUN/steps.log"
bash -c "$CAPTURE_CMD" > "$RUN/captures/after_topics.txt" 2>&1
stop_owned; step "recorder and launch stopped (cleanup.log)"
step end
