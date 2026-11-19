#!/usr/bin/env bash
# S4e harness template: the run-local safety net (from the tested S4d-E live watchdog). Armed
# BEFORE the launch. If the deadline passes before RUN/stopped.flag exists: owned-process cleanup,
# then the disabled restore. Writes its pid and deadline into RUN for the time checks.
# Usage: watchdog.sh <run dir> [deadline s]
RUN=$1; H=$(dirname "$(realpath "$0")"); source "$H/env.sh"; DEADLINE_S=${2:-$WATCHDOG_S}
end=$(( $(date +%s) + DEADLINE_S ))
echo "$end" > "$RUN/watchdog_deadline_epoch.txt"; echo $$ > "$RUN/watchdog.pid"
echo "$(date -u +%FT%TZ) watchdog armed: pid $$, deadline ${DEADLINE_S} s" >> "$RUN/watchdog_armed.txt"
while [ "$(date +%s)" -lt "$end" ]; do [ -f "$RUN/stopped.flag" ] && exit 0; sleep 1; done
[ -f "$RUN/stopped.flag" ] && exit 0
touch "$RUN/stopped.flag"
echo "$(date -u +%FT%TZ) watchdog: deadline reached; cleanup.sh, then restore.sh" >> "$RUN/watchdog.txt"
bash "$H/cleanup.sh" "$RUN" >> "$RUN/watchdog.txt" 2>&1
bash "$H/restore.sh" "$RUN" >> "$RUN/watchdog.txt" 2>&1
echo "$(date -u +%FT%TZ) watchdog: done" >> "$RUN/watchdog.txt"
