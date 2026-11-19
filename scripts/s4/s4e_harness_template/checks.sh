# S4e harness template: the refusals before the launch (sourced; each function returns non-zero
# with a reason on stdout). Wrong mode, installation, gates or ref, too little watchdog time or
# disk: nothing is launched.
check_mode() {
  case "$S4E_MODE" in shadow|live) return 0 ;; esac
  echo "S4E_MODE '$S4E_MODE' is not shadow or live"; return 1
}
check_install() {
  [ -n "$RUNINST" ] || { echo "RUNINST is not set"; return 1; }
  for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH; do echo "$v=${!v}"; done > "$RUN/env_run.txt"
  grep -q -- "$OLD_INSTALL" "$RUN/env_run.txt" && { echo "the installation $OLD_INSTALL is in the environment"; return 1; }
  grep -q -- "$RUNINST" "$RUN/env_run.txt" || { echo "the run installation $RUNINST is not in the environment"; return 1; }
  return 0
}
check_gates() {
  local G WANT; G=$(bash -c "$GATES_CMD" 2>&1 | tail -1); echo "gates (imported): $G" >> "$RUN/steps.log"
  WANT="False False False"; [ "$S4E_MODE" = live ] && WANT="True False False"
  [ "$(echo "$G" | cut -d' ' -f1-3)" = "$WANT" ] || { echo "$S4E_MODE needs the gates '$WANT', imported '$(echo "$G" | cut -d' ' -f1-3)'"; return 1; }
  case "$(echo "$G" | cut -d' ' -f4)" in "$RUNINST"/*) return 0 ;; esac
  echo "the gates were not imported from the run installation $RUNINST"; return 1
}
check_ref() {
  local CUR NS EBR
  [ -n "$BASELINE" ] || { echo "BASELINE is not set"; return 1; }
  [ -z "$(git -C "$WS" status --porcelain)" ] || { echo "the worktree of $WS is not clean"; return 1; }
  CUR=$(git -C "$WS" rev-parse --abbrev-ref HEAD)
  if [ "$S4E_MODE" = shadow ]; then
    [ "$(git -C "$WS" rev-parse HEAD)" = "$(git -C "$WS" rev-parse "$BASELINE")" ] || { echo "shadow: HEAD is not the disabled baseline $BASELINE"; return 1; }
    case "$CUR" in local/*) echo "shadow: a local enabling branch ($CUR) is checked out"; return 1 ;; esac
    return 0
  fi
  EBR=$(cat "$RUN/../enabling_branch.txt" 2>/dev/null)
  # shellcheck disable=SC2254
  case "$EBR" in $ENABLING_PATTERN) ;; *) echo "live: no recorded $ENABLING_PATTERN enabling branch (got '${EBR:-none}')"; return 1 ;; esac
  [ "$CUR" = "$EBR" ] || { echo "live: the checked-out branch '$CUR' is not the recorded enabling branch"; return 1; }
  [ -z "$(git -C "$WS" for-each-ref "refs/remotes/*/$EBR")" ] || { echo "live: the enabling branch has a remote-tracking ref (it must never be pushed)"; return 1; }
  NS=$(git -C "$WS" diff --numstat "$BASELINE" HEAD | tr '\t' ' ' | sort | tr '\n' ';')
  [ "$NS" = "1 1 src/spiderx_controller/spiderx_controller/m55_contract.py;1 1 src/spiderx_controller/test/m55_gate.py;" ] || { echo "live: the enabling commit is not exactly the two-line M55 patch ($NS)"; return 1; }
  return 0
}
wd_check() {
  local WP DL REM; WP=$(tr -dc '0-9' < "$RUN/watchdog.pid" 2>/dev/null); DL=$(tr -dc '0-9' < "$RUN/watchdog_deadline_epoch.txt" 2>/dev/null)
  if [ -z "$WP" ] || [ -z "$DL" ] || ! kill -0 "$WP" 2>/dev/null || ! tr '\0' ' ' < "/proc/$WP/cmdline" | grep -q "watchdog.sh $RUN"; then
    echo "time $1: the watchdog is not running"; return 1; fi
  REM=$(( DL - $(date +%s) ))
  [ "$REM" -ge $(( SESSION_CAP_S + AFTER_S )) ] && { echo "time $1: PASS: ${REM} s left >= ${SESSION_CAP_S} + ${AFTER_S} s"; return 0; }
  echo "time $1: FAIL: ${REM} s left < ${SESSION_CAP_S} + ${AFTER_S} s"; return 1
}
disk_check() {
  local DL T AV NEED; DL=$(tr -dc '0-9' < "$RUN/watchdog_deadline_epoch.txt" 2>/dev/null)
  T=$(( ${DL:-0} - $(date +%s) )); [ "$T" -lt 0 ] && T=0
  AV=$(bash -c "$DISK_FREE_CMD \"$(dirname "$BAG")\"" 2>/dev/null | tail -1 | tr -dc '0-9')
  NEED=$(( 2 * RATE_B_S * T + RESTORE_RESERVE_B )); [ "$NEED" -lt "$DISK_MIN_B" ] && NEED=$DISK_MIN_B
  [ -n "$AV" ] && [ "$AV" -ge "$NEED" ] && { echo "disk $1: PASS: ${AV} B free >= ${NEED} B"; return 0; }
  echo "disk $1: FAIL: ${AV:-unknown} B free < ${NEED} B (max(5 GiB, 2E + reserve), E over ${T} s)"; return 1
}
