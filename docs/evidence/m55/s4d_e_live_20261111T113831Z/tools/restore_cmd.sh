#!/usr/bin/env bash
# S4d-E LIVE restore (the tested S4d-M live restore_cmd.sh; only the run root, the baseline, the branch and
# the run installation changed). Fixed before enabling; not changed after the launch. Used by hand after
# the run and by the watchdog.
#  1. scripts/m61_restore_and_verify_disabled.sh: the exact disabled baseline 16d939f (evidence-only commits
#     on top of the validated executable 87198b9); the enabling ref recorded by enable_build.sh (only once
#     that branch exists) with --delete-enabling-branch (the script deletes it only after every verification
#     passed, only if never pushed, and only after preserving its commit and patch); the toolchain-only
#     INCREMENTAL rebuild of the normal installation (colcon build --symlink-install, st59site confined to the
#     build; build/, install/ and log/ are never deleted); the run's pid files (passed only if they exist:
#     before a launch, or after a partial failure, there may be nothing to stop; the leftover check still runs).
#  2. Only if DISABLED AND VERIFIED: back onto claude/spiderx-s4d-e-preparation, and only if that branch
#     still points at the baseline (the same commit: no file changes).
#  3. The RUN installation (the fresh enabled B1 bases, symlinked into the restored src/) in a fresh
#     environment (run_install_gates.sh; M15/E12), if it was built.
# Exit 0 only if all three hold.
R=/root/spiderx_evidence/s4d_e_live_20261111T113831Z
BASE=16d939f5a0e06afa3478093f0b491e3e8ce60673; BR=claude/spiderx-s4d-e-preparation
RUNINST=/root/spiderx_e_live_enabled_20261111T113831Z/install
B=$(cat $R/enabling_branch.txt 2>/dev/null)
ARGS=(); [ -n "$B" ] && ARGS+=(--enabling-ref "$B" --delete-enabling-branch)
[ -f $R/run/launch.pid ] && ARGS+=(--launch-pid-file $R/run/launch.pid)
[ -f $R/run/xvfb.pid ] && ARGS+=(--xvfb-pid-file $R/run/xvfb.pid)
echo "$(date -u +%FT%TZ) restore_cmd: base $BASE; enabling ref ${B:-none}; args ${ARGS[*]}"
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base $BASE \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh \
  --build-cmd 'PYTHONPATH=/opt/mm/st59site colcon build --symlink-install' "${ARGS[@]}"
RC1=$?
RC2=1
if [ $RC1 = 0 ] && [ "$(git rev-parse HEAD)" = $BASE ] && [ "$(git rev-parse refs/heads/$BR 2>/dev/null)" = $BASE ]; then
  git switch -q $BR && RC2=0
fi
echo "after restore: branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD); status entries $(git status --porcelain | wc -l); local/* branches $(git branch --list 'local/*' | wc -l)"
if [ -d $RUNINST ]; then bash $R/tools/run_install_gates.sh $RUNINST $R/run_install; RC3=$?
else echo "run installation $RUNINST not built: nothing to check"; RC3=0; fi
echo "restore exit $RC1; back on $BR at the baseline: $([ $RC2 = 0 ] && echo yes || echo NO); run installation gates exit $RC3"
[ $RC1 = 0 ] && [ $RC2 = 0 ] && [ $RC3 = 0 ]
