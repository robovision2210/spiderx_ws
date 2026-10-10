#!/usr/bin/env bash
# M6.1 one-cycle cleanup and proof of the disabled state (docs/M61_CLOUD_ONE_CYCLE_PLAN.md, E11).
# Run it after the cycle whatever happened: success, failure or interruption. It is idempotent.
#
#   scripts/m61_restore_and_verify_disabled.sh --base <disabled base branch or commit> \
#       --evidence <dir> [--enabling-ref <local enabling branch or commit>] \
#       [--launch-pid-file <run>/launch.pid] [--xvfb-pid-file <run>/xvfb.pid] \
#       [--ws <workspace>] [--ros-setup <file>] \
#       [--build-cmd <command>] [--verify-only] [--discard-uncommitted-src] \
#       [--delete-enabling-branch]
#
# Switching branches is not proof. These steps are, and each is recorded in
# <evidence>/restore_verify_<UTC>.txt:
#  1. Stop. With --launch-pid-file: SIGINT to that launch's process group (Ctrl+C), wait up to
#     60 s, then SIGKILL what is left; with --xvfb-pid-file: SIGTERM to the virtual display (it is
#     not in the launch's group). Then no simulator, bridge, spawner, launch, display or ros2-daemon
#     process may remain. This script's own process chain is excluded, so a shell whose command
#     line merely contains a pattern is not reported as a leftover.
#  2. Preserve, before any checkout or deletion: the enabling commit's id, `git show --stat`,
#     `git format-patch`, and the SHA-256 of its src/ diff against the base. Uncommitted src/
#     changes (an interrupted apply) are saved as a diff. They are discarded only with
#     --discard-uncommitted-src; otherwise the script stops.
#  3. Restore: check out the base. src/ must then equal the base exactly: no diff and no
#     untracked files.
#  4. Rebuild: --build-cmd (default `rm -rf build install && colcon build --symlink-install`).
#     log/ and the evidence directory are never removed.
#  5. Verify, in a FRESH environment (env -i, then --ros-setup, then <ws>/install/setup.bash):
#     a. Python imports spiderx_controller.m61_live_contract and m6_live_contract from <ws>, and
#        both gates are False;
#     b. `ros2 pkg prefix spiderx_controller` is <ws>/install/spiderx_controller;
#     c. ONLY IF (a) passed: `m6_gait_replay.py --live` exits 3 and prints HARD-DISABLED. The gate
#        refuses before any ROS initialization. Stdin is /dev/null, the domain is unused and the
#        traffic is localhost only, so even a wrong build could not reach the simulation;
#     d. the CALLING environment must not import an enabled module, or one from elsewhere. Open
#        a new shell, or re-source, if it does.
#  6. Only if 1-5 passed and --delete-enabling-branch was given: delete the local enabling branch.
#     It must be named local/m61-one-cycle-*, have no remote-tracking ref and not be the base.
# --verify-only skips the checkout in 3 and the rebuild in 4. The comparison in 3, and steps 1
# and 5, still run.
# Exit 0: disabled and verified. 1: a check failed (do not continue; fix and re-run). 2: usage.

set -u
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=$(cd "$SELF_DIR/.." && pwd)
BASE= EVIDENCE= ENABLING= PIDFILE= XVFBFILE= ROS_SETUP= VERIFY_ONLY=0 DISCARD=0 DELETE=0
BUILD_CMD='rm -rf build install && colcon build --symlink-install'
LIVE_DOMAIN=231                       # unused here; the disabled gate refuses before any ROS call
usage() { sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --base) BASE=$2; shift 2 ;;
    --evidence) EVIDENCE=$2; shift 2 ;;
    --enabling-ref) ENABLING=$2; shift 2 ;;
    --launch-pid-file) PIDFILE=$2; shift 2 ;;
    --xvfb-pid-file) XVFBFILE=$2; shift 2 ;;
    --ws) WS=$(cd "$2" && pwd) || usage; shift 2 ;;
    --ros-setup) ROS_SETUP=$2; shift 2 ;;
    --build-cmd) BUILD_CMD=$2; shift 2 ;;
    --verify-only) VERIFY_ONLY=1; shift ;;
    --discard-uncommitted-src) DISCARD=1; shift ;;
    --delete-enabling-branch) DELETE=1; shift ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1" >&2; usage ;;
  esac
done
[ -n "$BASE" ] && [ -n "$EVIDENCE" ] || usage
if [ -z "$ROS_SETUP" ]; then
  for f in /opt/ros/humble/setup.bash /opt/mm/activate.sh; do
    [ -f "$f" ] && { ROS_SETUP=$f; break; }
  done
fi
[ -n "$ROS_SETUP" ] && [ -f "$ROS_SETUP" ] || { echo "no ROS setup file (--ros-setup)" >&2; exit 2; }
mkdir -p "$EVIDENCE" || exit 2
EVIDENCE=$(cd "$EVIDENCE" && pwd)
UTC=$(date -u +%Y%m%dT%H%M%SZ)
REPORT=$EVIDENCE/restore_verify_$UTC.txt
FAILED=0
say() { echo "$*" | tee -a "$REPORT"; }
fail() { FAILED=1; say "FAIL: $*"; }
ok() { say "ok: $*"; }
git_ws() { git -C "$WS" "$@"; }

say "M6.1 restore and verify, $UTC, workspace $WS, base $BASE"
BASE_COMMIT=$(git_ws rev-parse --verify "$BASE^{commit}" 2>/dev/null) || {
  say "FAIL: base $BASE is not a commit"; exit 1; }
say "base commit $BASE_COMMIT"

# ---------------------------------------------------------------- 1. stop
ancestors() {   # this script's process chain: never a leftover
  local p=$$
  while [ -n "$p" ] && [ "$p" -gt 1 ]; do
    echo "$p"
    p=$(awk '/^PPid:/ {print $2}' "/proc/$p/status" 2>/dev/null)
  done
}
leftovers() {
  local skip pid ppid comm args
  skip=" $(ancestors | tr '\n' ' ') "
  ps -eo pid=,ppid=,comm=,args= | while read -r pid ppid comm args; do
    case "$skip" in *" $pid "*) continue ;; esac
    [ "$ppid" = "$$" ] && continue                     # this script's own pipeline members
    [[ "$args" == *m61_restore_and_verify_disabled* ]] && continue
    if [[ "$comm" =~ ^(ruby|Xvfb|gzserver|parameter_bridg|robot_state_pub|spawner|create)$ ]] ||
       [[ "$args" =~ (ign\ gazebo|gz\ sim|parameter_bridge|robot_state_publisher|ros2\ launch|_ros2_daemon|Xvfb|spawner|create\ -name|ros_gz) ]]; then
      echo "$pid $comm $args"
    fi
  done
}
say "--- 1. stop"
if [ -n "$PIDFILE" ]; then
  if [ -f "$PIDFILE" ]; then
    LP=$(cat "$PIDFILE")
    if pgrep -g "$LP" > /dev/null 2>&1; then
      kill -INT -- "-$LP" 2>/dev/null && say "SIGINT sent to process group $LP"
      for _ in $(seq 1 60); do pgrep -g "$LP" > /dev/null 2>&1 || break; sleep 1; done
      if pgrep -g "$LP" > /dev/null 2>&1; then
        kill -KILL -- "-$LP" 2>/dev/null; sleep 1
        say "process group $LP still present after 60 s: SIGKILL sent"
      fi
    else
      say "process group $LP already empty"
    fi
  else
    fail "launch pid file $PIDFILE not found"
  fi
fi
if [ -n "$XVFBFILE" ]; then
  if [ -f "$XVFBFILE" ]; then
    XP=$(cat "$XVFBFILE")
    if ps -p "$XP" -o comm= 2>/dev/null | grep -qx Xvfb; then
      kill -TERM "$XP" 2>/dev/null && say "SIGTERM sent to Xvfb $XP"
      for _ in $(seq 1 20); do ps -p "$XP" > /dev/null 2>&1 || break; sleep 0.5; done
      ps -p "$XP" > /dev/null 2>&1 && { kill -KILL "$XP" 2>/dev/null; say "Xvfb $XP: SIGKILL sent"; }
    else
      say "Xvfb $XP not running"
    fi
  else
    fail "Xvfb pid file $XVFBFILE not found"
  fi
fi
( source "$ROS_SETUP" > /dev/null 2>&1 && ros2 daemon stop > /dev/null 2>&1 ) || true
LEFT=$(leftovers)
if [ -n "$LEFT" ]; then fail "simulation processes remain:"; say "$LEFT"; else ok "no simulation process"; fi

# ---------------------------------------------------------------- 2. preserve
say "--- 2. preserve"
if [ -n "$ENABLING" ]; then
  if EC=$(git_ws rev-parse --verify "$ENABLING^{commit}" 2>/dev/null); then
    echo "$ENABLING $EC" > "$EVIDENCE/enabling_commit.txt"
    git_ws show --stat --format=fuller "$EC" > "$EVIDENCE/enabling_commit_show.txt"
    git_ws format-patch -1 --stdout "$EC" > "$EVIDENCE/enabling_commit.patch"
    git_ws diff "$BASE_COMMIT" "$EC" -- src/ > "$EVIDENCE/enabling_src_vs_base.diff"
    say "enabling $ENABLING = $EC; src diff vs base sha256 $(sha256sum < "$EVIDENCE/enabling_src_vs_base.diff" | cut -d' ' -f1)"
    git_ws diff --stat "$BASE_COMMIT" "$EC" -- src/ | tee -a "$REPORT"
  else
    say "enabling ref $ENABLING not found (never created, or already deleted)"
  fi
fi
git_ws status --short > "$EVIDENCE/worktree_status_before_restore.txt"
if ! git_ws diff --quiet HEAD -- src/ || ! git_ws diff --cached --quiet HEAD -- src/; then
  git_ws diff HEAD -- src/ > "$EVIDENCE/uncommitted_src_$UTC.diff"
  say "uncommitted src/ changes saved: uncommitted_src_$UTC.diff"
  if [ "$DISCARD" = 1 ] && [ "$VERIFY_ONLY" = 0 ]; then
    git_ws checkout HEAD -- src/ && git_ws reset -q HEAD -- src/ && say "uncommitted src/ changes discarded"
  else
    fail "uncommitted src/ changes (saved); re-run with --discard-uncommitted-src to discard them"
  fi
fi

# ---------------------------------------------------------------- 3. restore
say "--- 3. restore"
if [ "$VERIFY_ONLY" = 0 ] && [ "$FAILED" = 0 ]; then
  if git_ws show-ref --verify --quiet "refs/heads/$BASE"; then
    git_ws switch -q "$BASE" 2>&1 | tee -a "$REPORT"
  else
    git_ws switch -q --detach "$BASE_COMMIT" 2>&1 | tee -a "$REPORT"
  fi
fi
say "HEAD $(git_ws rev-parse HEAD) ($(git_ws rev-parse --abbrev-ref HEAD))"
if git_ws diff --quiet "$BASE_COMMIT" -- src/ && [ -z "$(git_ws ls-files --others --exclude-standard -- src/)" ]; then
  ok "src/ equals the base $BASE_COMMIT (no diff, no untracked file)"
else
  fail "src/ differs from the base $BASE_COMMIT:"
  git_ws diff --stat "$BASE_COMMIT" -- src/ | tee -a "$REPORT"
  git_ws ls-files --others --exclude-standard -- src/ | tee -a "$REPORT"
fi

# ---------------------------------------------------------------- 4. rebuild
say "--- 4. rebuild"
if [ "$VERIFY_ONLY" = 0 ] && [ "$FAILED" = 0 ]; then
  if ( cd "$WS" && env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
         bash --noprofile --norc -c "source '$ROS_SETUP' > /dev/null 2>&1; $BUILD_CMD" ) > "$EVIDENCE/rebuild_$UTC.log" 2>&1; then
    ok "rebuilt ($BUILD_CMD); log rebuild_$UTC.log"
  else
    fail "rebuild failed; see rebuild_$UTC.log"
  fi
else
  say "skipped"
fi

# ---------------------------------------------------------------- 5. verify
say "--- 5. verify (fresh environment)"
IMPORT_CHECK='
import importlib, os, sys
ws = os.path.realpath(sys.argv[1])
bad = 0
for mod, attr in (("spiderx_controller.m61_live_contract", "M61_LIVE_DISPATCH_ENABLED"),
                  ("spiderx_controller.m6_live_contract", "LIVE_DISPATCH_ENABLED")):
    m = importlib.import_module(mod)
    path = os.path.realpath(m.__file__)
    val = getattr(m, attr)
    inside = path.startswith(ws + os.sep)
    print(f"{mod}.{attr} = {val!r} imported from {path} (inside the workspace: {inside})")
    bad |= (val is not False) or not inside
sys.exit(1 if bad else 0)
'
# Python puts the current directory first on sys.path for `-c`: run every check from / so a
# package directory under the caller's cwd (e.g. src/spiderx_controller) cannot shadow the
# installed module.
fresh() {
  ( cd / && env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$LIVE_DOMAIN \
    bash --noprofile --norc -c "source '$ROS_SETUP' > /dev/null 2>&1; source '$WS/install/setup.bash' > /dev/null 2>&1; $1" )
}
if [ ! -f "$WS/install/setup.bash" ]; then
  fail "no $WS/install/setup.bash: nothing to verify"
  GATES_OK=0
else
  OUT=$(fresh "python3 -c '$IMPORT_CHECK' '$WS'" 2>&1); RC=$?
  say "$OUT"
  if [ $RC = 0 ]; then GATES_OK=1; ok "a. both gates False in the imported modules, from this workspace"
  else GATES_OK=0; fail "a. imported gate module enabled, unreadable or from outside the workspace"; fi
  PREFIX=$(fresh "ros2 pkg prefix spiderx_controller" 2>&1 | tail -1)
  if [ "$(realpath -m "$PREFIX")" = "$(realpath -m "$WS/install/spiderx_controller")" ]; then
    ok "b. ros2 pkg prefix spiderx_controller = $PREFIX"
  else
    fail "b. ros2 pkg prefix spiderx_controller = $PREFIX (expected $WS/install/spiderx_controller)"
  fi
  if [ "$GATES_OK" = 1 ]; then
    LIVE=$(fresh "timeout 120 ros2 run spiderx_controller m6_gait_replay.py --live --domain-id $LIVE_DOMAIN < /dev/null" 2>&1); LRC=$?
    say "$LIVE"
    if [ $LRC = 3 ] && grep -q "HARD-DISABLED" <<< "$LIVE"; then ok "c. --live refused, exit 3, HARD-DISABLED"
    else fail "c. --live exit $LRC (expected 3 with HARD-DISABLED)"; fi
  else
    fail "c. --live NOT run: the gate is not verified disabled"
  fi
fi
if ( cd / && python3 -c 'import spiderx_controller' ) > /dev/null 2>&1; then
  OUT=$(cd / && python3 -c "$IMPORT_CHECK" "$WS" 2>&1); RC=$?
  say "calling environment: $OUT"
  if [ $RC = 0 ]; then ok "d. the calling environment imports the disabled modules of this workspace"
  else fail "d. the calling environment imports an enabled module or one from elsewhere: open a new shell or re-source"; fi
else
  ok "d. the calling environment cannot import spiderx_controller (not sourced)"
fi

# ---------------------------------------------------------------- 6. optional deletion
say "--- 6. enabling branch"
if [ "$DELETE" = 1 ]; then
  if [ "$FAILED" != 0 ]; then
    say "not deleted: a check failed"
  elif [ -z "$ENABLING" ] || ! git_ws show-ref --verify --quiet "refs/heads/$ENABLING"; then
    say "no local branch ${ENABLING:-<none>} to delete"
  elif [[ "$ENABLING" != local/m61-one-cycle-* ]] || [ "$ENABLING" = "$BASE" ]; then
    fail "refusing to delete $ENABLING: not a local/m61-one-cycle-* branch, or it is the base"
  elif [ -n "$(git_ws for-each-ref "refs/remotes/*/$ENABLING")" ]; then
    fail "refusing to delete $ENABLING: a remote-tracking ref exists (it was pushed)"
  elif [ ! -s "$EVIDENCE/enabling_commit.patch" ]; then
    fail "refusing to delete $ENABLING: its identity is not preserved in $EVIDENCE"
  else
    git_ws branch -D "$ENABLING" 2>&1 | tee -a "$REPORT"
  fi
else
  say "kept (no --delete-enabling-branch)"
fi
git_ws branch --list 'local/m61-one-cycle-*' | sed 's/^/local branch: /' | tee -a "$REPORT"

if [ "$FAILED" = 0 ]; then say "VERDICT: DISABLED AND VERIFIED"; exit 0; fi
say "VERDICT: NOT VERIFIED - do not continue; fix and re-run"
exit 1
