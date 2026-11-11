#!/usr/bin/env bash
# S4d-M LIVE enabling and B1 build (OD12), run ONCE, after the watchdog is armed (OD8: the run script
# refuses to launch unless the watchdog still has the 90-minute script cap + 15 min left).
#  1. From the clean preparation branch at the exact disabled baseline 9cc1f68: a NEW local branch
#     local/m55-s4d-m-<UTC>, written to R/enabling_branch.txt as soon as it exists (restore_cmd.sh reads it).
#  2. Only the recorded two-line patch docs/patches/m55_enable_s2.UNAPPLIED.patch (sha256 d1dacac5...):
#     hash, scope (exactly the M55 gate and its test pin), git apply --check; applied and committed LOCALLY.
#     Never pushed: no upstream, no remote-tracking ref, not on the remote (checked and saved).
#  3. B1: a fresh enabled build in NEW bases /root/spiderx_live_enabled_20261111T052817Z (they must not exist),
#     --symlink-install, from a toolchain-only env -i shell with the r2 overlay; st59site on PYTHONPATH for
#     the build only (the documented procedure, as the batch-2 fresh disabled build).
#  4. The enabled M5.5 tests (the seven files the S4c live attempts ran), against the new installation.
#  5. From the new installation: M55 True, M61 False, M6.0-D False, imported through its symlinks into
#     src/; library fingerprint 62f8a100...; the enabled library.json = the disabled 917fc4d7....
# Every step is timed (enabled/summary.txt). Exit 0 only if all pass; otherwise the caller restores
# (restore_cmd.sh) and stops without a launch. Nothing is deleted.
# Usage: enable_build.sh <UTC stamp>
set -u
R=/root/spiderx_evidence/s4d_live_20261111T052817Z; WS=/home/user/spiderx_ws
BASE=9cc1f6844de11abc630387d4c6c441a623463094; PREP=claude/spiderx-s4d-preparation
EB=/root/spiderx_live_enabled_20261111T052817Z; O=$R/enabled
P=docs/patches/m55_enable_s2.UNAPPLIED.patch; PH=d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236
SCOPE="1 1 src/spiderx_controller/spiderx_controller/m55_contract.py;1 1 src/spiderx_controller/test/m55_gate.py;"
TESTS="test/test_m55_command.py test/test_m55_gait.py test/test_m55_locomotion.py test/test_m55_rate_rule.py test/test_m55_rclpy_shutdown.py test/test_m55_review.py test/test_m55_ros.py"
[ -n "${1:-}" ] || { echo "usage: enable_build.sh <UTC stamp>"; exit 2; }
BR=local/m55-s4d-m-$1
mkdir -p $O; S=$O/summary.txt; t0=$(date +%s)
T() { echo "[$(date -u +%T) +$(( $(date +%s) - t0 )) s] $*" | tee -a $S; }
die() { T "FAIL: $*"; exit 1; }
fresh() { env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  EB="$EB" WS="$WS" O="$O" R="$R" TESTS="$TESTS" bash --noprofile --norc -c "source /opt/mm/activate.sh >/dev/null 2>&1; source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash; $1"; }
cd $WS
T "start: branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD) worktree entries $(git status --porcelain | wc -l)"
[ "$(git rev-parse --abbrev-ref HEAD)" = $PREP ] && [ "$(git rev-parse HEAD)" = $BASE ] && [ -z "$(git status --porcelain)" ] \
  || die "not on the clean preparation branch at the baseline $BASE"
[ ! -e $EB ] || die "$EB already exists (the B1 bases must be new)"
[ -z "$(git branch --list 'local/*')" ] || die "a local/* branch already exists"
h=$(sha256sum < $P | cut -c1-64); [ "$h" = $PH ] || die "patch hash $h"
NS=$(git apply --numstat $P | tr '\t' ' ' | sort | tr '\n' ';'); [ "$NS" = "$SCOPE" ] || die "patch scope $NS"
git apply --check $P || die "git apply --check"
T "patch $P: sha256 $h, scope $NS, applies"
git switch -q -c $BR $BASE || die "branch creation"
echo $BR > $R/enabling_branch.txt
T "created $BR from $BASE (recorded in enabling_branch.txt)"
git apply $P || die "git apply"
git add -- src/spiderx_controller/spiderx_controller/m55_contract.py src/spiderx_controller/test/m55_gate.py || die "git add"
git commit -q -F - <<MSG || die "commit"
LOCAL ONLY, NEVER PUSHED: enable M5.5 dispatch for the one S4d-M live attempt

Applies docs/patches/m55_enable_s2.UNAPPLIED.patch (sha256 $PH): the M55 gate and
its test pin, two lines. M61 and M6.0-D stay False. Deleted by
scripts/m61_restore_and_verify_disabled.sh only after DISABLED AND VERIFIED.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01QzFL2kguRig19KtzFfqEm3
MSG
EC=$(git rev-parse HEAD); echo "$BR $EC" > $O/enabling_commit.txt
git show --stat --format=fuller $EC > $O/enabling_commit_show.txt; git format-patch -1 --stdout $EC > $O/enabling_commit.patch
git diff $BASE $EC > $O/enabling_diff.txt
NS2=$(git diff --numstat $BASE $EC | tr '\t' ' ' | sort | tr '\n' ';'); [ "$NS2" = "$SCOPE" ] || die "commit scope $NS2"
UP=$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null || echo none)
REM=$(git for-each-ref "refs/remotes/*/$BR" | wc -l); LSR=$(timeout 30 git ls-remote --heads origin "$BR" 2>&1 | wc -l)
echo "$(date -u +%FT%TZ) $BR $EC: upstream $UP; remote-tracking refs $REM; ls-remote heads matching $LSR" | tee $O/never_pushed_check.txt
[ "$UP" = none ] && [ "$REM" = 0 ] && [ "$LSR" = 0 ] || die "the enabling branch has an upstream or a remote ref"
G=$(grep -h -E "^(M55_LOCOMOTION_DISPATCH_ENABLED|M61_LIVE_DISPATCH_ENABLED|LIVE_DISPATCH_ENABLED) = " src/spiderx_controller/spiderx_controller/m55_contract.py src/spiderx_controller/spiderx_controller/m61_live_contract.py src/spiderx_controller/spiderx_controller/m6_live_contract.py | tr '\n' ';')
T "enabling commit $EC: diff $NS2; source gates: $G"
[ "$G" = "M55_LOCOMOTION_DISPATCH_ENABLED = True;M61_LIVE_DISPATCH_ENABLED = False;LIVE_DISPATCH_ENABLED = False;" ] || die "source gates"
T "B1 build start (new bases $EB)"
fresh 'for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do echo "$v=${!v}"; done > $O/env_build.txt
  cd $WS && PYTHONPATH=/opt/mm/st59site colcon --log-base $EB/log build --build-base $EB/build --install-base $EB/install --symlink-install' > $O/build.log 2>&1; BRC=$?
T "B1 build exit $BRC: $(grep -E '^Summary' $O/build.log | tail -1)"
[ $BRC = 0 ] || die "B1 build"
grep -q "/home/user/spiderx_ws/install" $O/env_build.txt && die "the existing workspace installation was in the build environment"
T "enabled M5.5 tests start"
fresh 'source $EB/install/setup.bash; cd $WS/src/spiderx_controller && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=232 timeout 900 python3 -m pytest -q -p no:cacheprovider $TESTS' > $O/m55_tests.txt 2>&1; TRC=$?
T "enabled M5.5 tests exit $TRC: $(tail -1 $O/m55_tests.txt)"
[ $TRC = 0 ] || die "enabled M5.5 tests"
fresh 'source $EB/install/setup.bash; cd / && PYTHONDONTWRITEBYTECODE=1 python3 -c "
import importlib, os
for mod, name in ((\"m55_contract\", \"M55_LOCOMOTION_DISPATCH_ENABLED\"), (\"m61_live_contract\", \"M61_LIVE_DISPATCH_ENABLED\"), (\"m6_live_contract\", \"LIVE_DISPATCH_ENABLED\")):
    m = importlib.import_module(\"spiderx_controller.\" + mod)
    print(\"gate\", name, getattr(m, name), m.__file__, os.path.realpath(m.__file__))"
  PYTHONDONTWRITEBYTECODE=1 python3 $WS/docs/evidence/m55/s2_fixed_base_20261029T100210Z/tools/library_fingerprint.py > $O/library_fingerprint.json
  PYTHONDONTWRITEBYTECODE=1 python3 $R/tools/library_json.py $O/library.json' > $O/imports.txt 2>&1
cat $O/imports.txt | tee -a $S
OK=1
for x in "M55_LOCOMOTION_DISPATCH_ENABLED True" "M61_LIVE_DISPATCH_ENABLED False" "LIVE_DISPATCH_ENABLED False"; do
  grep -q "^gate $x $EB/install/spiderx_controller/lib/python3.11/site-packages/spiderx_controller/[a-z0-9_]*\.py /home/user/spiderx_ws/src/spiderx_controller/spiderx_controller/[a-z0-9_]*\.py$" $O/imports.txt || OK=0; done
grep -q '"goals": 54, "fingerprints_sha256": "62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732", "m55_gate_imported": true' $O/library_fingerprint.json || OK=0
LJ=$(sha256sum < $O/library.json | cut -c1-64)
T "library: $(cat $O/library_fingerprint.json); enabled library.json $LJ (disabled 917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e)"
[ "$LJ" = 917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e ] || OK=0
[ $OK = 1 ] || die "enabled imports or library identity"
T "ENABLE AND BUILD: PASS ($(( $(date +%s) - t0 )) s)"
