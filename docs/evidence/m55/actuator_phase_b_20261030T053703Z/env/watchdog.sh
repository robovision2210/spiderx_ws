#!/usr/bin/env bash
# Phase B run-local safety net (not part of the frozen harness): if the session is interrupted,
# stop this launch through the harness once the deadline passes. Exits when RUN/stopped.flag exists.
RUN=$1; DEADLINE_S=${2:-1500}
EVT=$(dirname "$RUN")/tools
end=$(( $(date +%s) + DEADLINE_S ))
while [ "$(date +%s)" -lt "$end" ]; do [ -f "$RUN/stopped.flag" ] && exit 0; sleep 5; done
[ -f "$RUN/stopped.flag" ] && exit 0
touch "$RUN/stopped.flag"
echo "$(date -u +%FT%TZ) watchdog: deadline ${DEADLINE_S}s reached; running stage_stop.sh" >> "$RUN/watchdog.txt"
bash "$EVT/stage_stop.sh" "$RUN" >> "$RUN/watchdog.txt" 2>&1
