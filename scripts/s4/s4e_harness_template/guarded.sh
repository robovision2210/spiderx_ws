#!/usr/bin/env bash
# S4e harness template: the session with cleanup AND restore armed before anything starts (from
# the tested S4d-E guarded wrapper). Whatever happens (success, refusal, failure, SIGINT, SIGTERM,
# an interruption before any pid file exists), the exit trap stops what this run owns, then
# restores the disabled state. The watchdog must be armed first (watchdog.sh, in the background).
# Usage: guarded.sh <session dir> <bag dir>
SESSION=$1; BAG=$2; RUN=$SESSION/run; H=$(dirname "$(realpath "$0")")
mkdir -p "$RUN"
finish() {
  trap - EXIT INT TERM
  [ -n "$CHILD" ] && kill -TERM "$CHILD" 2>/dev/null && wait "$CHILD" 2>/dev/null
  touch "$RUN/stopped.flag"
  bash "$H/cleanup.sh" "$RUN" >> "$RUN/cleanup.log" 2>&1
  bash "$H/restore.sh" "$RUN" >> "$RUN/restore.log" 2>&1
}
trap finish EXIT
trap 'exit 130' INT TERM
bash "$H/s4e_session.sh" "$SESSION" "$BAG" &
CHILD=$!
wait "$CHILD"; RC=$?; CHILD=
exit $RC
