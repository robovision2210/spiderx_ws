#!/usr/bin/env bash
# S4d-E shadow input checks (read-only), BEFORE the watchdog and the one launch. Ends with
# "INPUT VERDICT: PASS" or "INPUT VERDICT: FAIL: <items>".
R=$(dirname "$(dirname "$(realpath "$0")")"); WS=/home/user/spiderx_ws
ESHA=87198b951506b35ee4d95f3c124c0327a9647ae6; EI=/root/spiderx_fresh_disabled_20261111T104338Z/install
FU=/root/spiderx_evidence/s4d_e_fresh_disabled_20261111T104338Z; F=""
cd $WS
echo "== $(date -u +%FT%TZ) repository"
echo "branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD) origin $(git rev-parse origin/claude/spiderx-s4d-e-preparation) worktree entries $(git status --porcelain | wc -l) local/* $(git branch --list 'local/*' | wc -l)"
[ "$(git rev-parse --abbrev-ref HEAD)" = claude/spiderx-s4d-e-preparation ] && [ "$(git rev-parse HEAD)" = $ESHA ] \
  && [ "$(git rev-parse origin/claude/spiderx-s4d-e-preparation)" = $ESHA ] && [ -z "$(git status --porcelain)" ] \
  && [ -z "$(git branch --list 'local/*')" ] && echo "PASS the validated E SHA, clean, = origin, no local/* branch" || F="$F repo"
git grep -n -E "^(M61_LIVE_DISPATCH_ENABLED|LIVE_DISPATCH_ENABLED|M55_LOCOMOTION_DISPATCH_ENABLED) = " -- src/ | cut -d: -f3- | tr '\n' ';'; echo
[ "$(git grep -h -E "^(M61_LIVE_DISPATCH_ENABLED|LIVE_DISPATCH_ENABLED|M55_LOCOMOTION_DISPATCH_ENABLED) = False" -- src/ | wc -l)" = 3 ] && echo "PASS source gates all False" || F="$F source_gates"
echo "== step 6 (fresh disabled validation at the same SHA)"
grep -q "head $ESHA worktree 0" $FU/fresh/build_and_test_summary.txt && grep -q "^Summary: 1993 tests, 0 errors, 0 failures, 0 skipped" $FU/fresh/build_and_test_summary.txt \
  && grep -q "COLLECTION VERDICT: PASS" $FU/fresh/build_and_test_summary.txt && grep -q "INSTALLATION A_fresh VERDICT: PASS" $FU/install_A_check.txt \
  && grep -q "INSTALLATION B_workspace VERDICT: PASS" $FU/install_B_check.txt && echo "PASS step 6 at $ESHA: 1993/0/0/0, both installations PASS" || F="$F step6"
echo "== the run installation $EI: python modules are symlinks into src/ equal to their HEAD blobs"
n=0; bad=0
for l in $(find $EI/spiderx_controller/lib/python3.11/site-packages/spiderx_controller -maxdepth 1 -name "*.py" -type l); do
  t=$(realpath $l); p=${t#$WS/}; n=$((n+1))
  [ "$(git hash-object $t)" = "$(git rev-parse HEAD:$p 2>/dev/null)" ] || { echo "DIFFERS $p"; bad=$((bad+1)); }; done
echo "$n symlinked modules, $bad differ"; [ $n -gt 0 ] && [ $bad = 0 ] && echo "PASS run installation = HEAD" || F="$F run_install"
echo "== r2 overlay libraries vs provenance"
for f in libgz_ros2_control-system.so libgz_hardware_plugins.so; do p=/root/spiderx_gz_overlay_ws_r2/install/gz_ros2_control/lib/$f
  h=$(sha256sum < $p | cut -c1-64); grep -q "^library $h $p" /root/spiderx_gz_overlay_ws_r2/OVERLAY_PROVENANCE.txt && echo "PASS $f ${h:0:16}" || { echo "FAIL $f"; F="$F overlay"; }; done
h=$(sha256sum < src/spiderx_controller/config/m55_locomotion.yaml | cut -c1-64); [ "$h" = ab0dccd43fefe877fac21e1c51d1fdd0ae197ac456b8c42502b0bb7ba5e42a93 ] && echo "PASS config ab0dccd4" || F="$F config"
echo "== E tooling = HEAD blobs"
for p in scripts/s4/s4d_e_driver.py scripts/s4/score_s4d_e.py scripts/s4/s4c_driver.py scripts/s4/s4a_driver.py scripts/s3/s3_driver.py scripts/s2/s2_driver.py scripts/s4/score_s4d.py scripts/m61_restore_and_verify_disabled.sh; do
  [ "$(git hash-object $p)" = "$(git rev-parse HEAD:$p)" ] && echo "$(git hash-object $p | cut -c1-12) $p = HEAD" || { echo "DIFFERS $p"; F="$F tool_$p"; }; done
echo "== host"
P=$(pgrep -af "ign gazebo|gz sim|ros2 launch|controller_manager|m55_|teleop|Xvfb|s4d_|bag record|watchdog.sh" | grep -v -E "pgrep|input_checks"); [ -z "$P" ] && echo "PASS no relevant process" || { echo "FAIL processes: $P"; F="$F processes"; }
[ ! -e /tmp/.X77-lock ] && echo "PASS display :77 free" || F="$F display"
[ ! -e $R/run ] && [ ! -e $R/bag ] && echo "PASS no run or bag directory yet" || F="$F run_exists"
grep -q "^- E shadow launch used: NO " /root/spiderx_evidence/s4d_e_batch_journal.md && echo "PASS the E shadow launch is unused (journal)" || F="$F journal"
python3 - "$R" <<'PY'
import os, sys
st = os.statvfs(sys.argv[1]); avail = st.f_bavail * st.f_frsize
E = 2 * 30.8e6 / 60 * 7200; need = max(5 * 2**30, 2 * E + 0.1e9)
print(f'{"PASS" if avail >= need else "FAIL"} OD16 now: {avail} B free; need {need:.0f} B (T 7200 s; 2x 30.8 MB/min planning assumption)')
PY
[ $? = 0 ] || F="$F od16"
[ -z "$F" ] && echo "INPUT VERDICT: PASS" || echo "INPUT VERDICT: FAIL:$F"
