#!/usr/bin/env bash
# S4c level-2 shadow run-local safety net (the Phase C watchdog): if the session is interrupted and the
# deadline passes, stop everything the run started (cleanup.sh, which ends with stage_stop.sh), then
# verify the disabled state (restore_cmd.sh: --verify-only; nothing is enabled in this run).
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
