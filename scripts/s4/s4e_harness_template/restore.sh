#!/usr/bin/env bash
# S4e harness template: the disabled restore (from the tested S4d-E restore_cmd.sh). Run by the
# guarded wrapper's exit trap and by the watchdog, also when no pid file exists yet (an
# interruption before the launch). Shadow: --verify-only (nothing was enabled). Live: the full
# restore with the recorded local/m55-s4e-h-* branch and --delete-enabling-branch (the script
# deletes it only after every check passed, only if it was never pushed). Idempotent.
# Usage: restore.sh <run dir>
RUN=$1; H=$(dirname "$(realpath "$0")"); source "$H/env.sh"
[ -f "$RUN/restore_done.flag" ] && { echo "[$(date -u +%T)] restore already done"; exit 0; }
EBR=$(cat "$RUN/../enabling_branch.txt" 2>/dev/null)
ARGS=(--base "$BASELINE" --evidence "$RUN/../restore")
if [ "$S4E_MODE" = live ] && [ -n "$EBR" ]; then ARGS+=(--enabling-ref "$EBR" --delete-enabling-branch)
else ARGS+=(--verify-only); fi
[ -f "$RUN/launch.pid" ] && ARGS+=(--launch-pid-file "$RUN/launch.pid")
echo "[$(date -u +%T)] restore: $RESTORE_CMD ${ARGS[*]}"
bash -c "$RESTORE_CMD $(printf '%q ' "${ARGS[@]}")"; RC=$?
echo "[$(date -u +%T)] restore exit $RC"
[ $RC = 0 ] && touch "$RUN/restore_done.flag"
exit $RC
