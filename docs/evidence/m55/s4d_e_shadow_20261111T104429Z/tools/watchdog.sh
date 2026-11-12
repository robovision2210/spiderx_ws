#!/usr/bin/env bash
# S4d-M shadow run-local safety net (the S4c level-2 shadow watchdog): if the session is interrupted
# and the deadline passes, stop everything the run owns (cleanup.sh, which ends with stage_stop.sh),
# then verify the disabled state (restore_cmd.sh: --verify-only and the run installation's gates;
# nothing is enabled in this run). Exits without acting when RUN/stopped.flag exists.
# S4d changes (OD8): the deadline defaults to 7200 s, counted from this script's start, which is
# before the launch; the deadline (epoch s) and this script's pid are written into RUN, so the run
# script can check before the launch and again before the first key that the remaining time covers
# the 90-minute script cap plus 15 min (stop, cleanup, restore and verification).
RUN=$1; DEADLINE_S=${2:-7200}
EVT=$(dirname "$RUN")/tools
end=$(( $(date +%s) + DEADLINE_S ))
echo "$end" > "$RUN/watchdog_deadline_epoch.txt"; echo $$ > "$RUN/watchdog.pid"
echo "$(date -u +%FT%TZ) watchdog armed: pid $$, deadline ${DEADLINE_S}s, epoch $end ($(date -u -d @$end +%FT%TZ))" >> "$RUN/watchdog_armed.txt"
while [ "$(date +%s)" -lt "$end" ]; do [ -f "$RUN/stopped.flag" ] && exit 0; sleep 5; done
[ -f "$RUN/stopped.flag" ] && exit 0
touch "$RUN/stopped.flag"
echo "$(date -u +%FT%TZ) watchdog: deadline ${DEADLINE_S}s reached; cleanup.sh, then restore_cmd.sh" >> "$RUN/watchdog.txt"
bash "$EVT/cleanup.sh" "$RUN" >> "$RUN/watchdog.txt" 2>&1
bash "$EVT/restore_cmd.sh" >> "$RUN/watchdog.txt" 2>&1
echo "$(date -u +%FT%TZ) watchdog: done" >> "$RUN/watchdog.txt"
