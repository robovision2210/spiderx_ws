#!/usr/bin/env bash
# Phase C run-local safety net (the Phase B watchdog, extended): if the session is interrupted and
# the deadline passes, stop everything the run started (the tested S2 cleanup.sh, which ends with
# stage_stop.sh), then restore and verify the disabled state (restore_cmd.sh).
# Exits without acting when RUN/stopped.flag exists.
RUN=$1; DEADLINE_S=${2:-3600}
EVT=$(dirname "$RUN")/tools
end=$(( $(date +%s) + DEADLINE_S ))
while [ "$(date +%s)" -lt "$end" ]; do [ -f "$RUN/stopped.flag" ] && exit 0; sleep 5; done
[ -f "$RUN/stopped.flag" ] && exit 0
touch "$RUN/stopped.flag"
echo "$(date -u +%FT%TZ) watchdog: deadline ${DEADLINE_S}s reached; cleanup.sh, then restore_cmd.sh" >> "$RUN/watchdog.txt"
bash "$EVT/cleanup.sh" "$RUN" >> "$RUN/watchdog.txt" 2>&1
bash "$EVT/restore_cmd.sh" >> "$RUN/watchdog.txt" 2>&1
echo "$(date -u +%FT%TZ) watchdog: done" >> "$RUN/watchdog.txt"
