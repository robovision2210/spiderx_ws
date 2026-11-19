#!/usr/bin/env bash
# POST-MERGE copy (2026-11-19) of the tested S4d-M/S4d-E fresh-build tool: ONE fresh DISABLED build and full
# suite of the S4e work clone (main e7ffd20 + the uncommitted S4e files), with NEW colcon bases. Changes: the workspace is the clone; any old
# SpiderX installation path in the environment refuses; the direct tests get SPIDERX_REPO=<clone> (their
# documented override; the default is the old checkout) and their full output is kept per suite.
# The bases are new, uniquely named and previously nonexistent, outside the tracked tree; the
# existing build/, install/ and log/ are not touched and nothing is deleted. Each step runs in a
# fresh shell (env -i): the documented toolchain, then the verified r2 overlay local_setup. The
# existing SpiderX installation is never sourced, and any of its paths in the environment refuses
# the step. st59site is on PYTHONPATH for the build command only; tests use
# PYTEST_DISABLE_PLUGIN_AUTOLOAD=1. Usage: fresh_build_test.sh <follow-up root> <fresh base dir>
FU=$1; F=$2; WS=/root/spiderx_s4e_20261119T063214Z/spiderx_ws; D=$FU/fresh
OLD='/home/user/spiderx_ws|/root/spiderx_fresh_disabled_|/root/spiderx_live_enabled_|/root/spiderx_e_live_enabled_|/root/spiderx_postmerge_'
[ -e "$F" ] && { echo "REFUSED: $F exists (bases must be new)"; exit 2; }
mkdir -p "$F" "$D"
fresh() { env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  F="$F" WS="$WS" D="$D" FU="$FU" OLD="$OLD" bash --noprofile --norc -c "source /opt/mm/activate.sh >/dev/null 2>&1; source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash; $1"; }
envdump='for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do echo "$v=${!v}"; done'
cd $WS
echo "start $(date -u +%FT%TZ) branch $(git rev-parse --abbrev-ref HEAD) head $(git rev-parse HEAD) worktree $(git status --porcelain | wc -l)"
echo "bases: build $F/build install $F/install log $F/log (created now; previously nonexistent)"
date +%s > $D/t0.txt
fresh "$envdump > \$D/env_build.txt
  if grep -q -E \"\$OLD\" \$D/env_build.txt; then echo 'REFUSED: an old SpiderX checkout or installation is in the build environment'; exit 2; fi
  cd \$WS && PYTHONPATH=/opt/mm/st59site colcon --log-base \$F/log build --build-base \$F/build --install-base \$F/install --symlink-install > \$D/build.log 2>&1; echo \"build exit=\$?\"" || exit 2
fresh "source \$F/install/setup.bash; $envdump > \$D/env_test.txt
  if grep -q -E \"\$OLD\" \$D/env_test.txt; then echo 'REFUSED: an old SpiderX checkout or installation is in the test environment'; exit 2; fi
  cd \$WS && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base \$F/log test --build-base \$F/build --install-base \$F/install --test-result-base \$F/build --event-handlers console_direct- > \$D/test.log 2>&1; echo \"test exit=\$?\"
  colcon --log-base \$F/log test-result --test-result-base \$F/build --verbose > \$D/colcon_test_result.txt 2>&1; echo \"result exit=\$?\"
  tail -1 \$D/colcon_test_result.txt
  python3 \$FU/tools/collection_check.py \$F \$(cat \$D/t0.txt) \$D/colcon_test_result.txt > \$D/collection_check.txt 2>&1
  tail -2 \$D/collection_check.txt" || exit 2
fresh "source \$F/install/setup.bash; cd \$WS; export SPIDERX_REPO=\$WS
  echo \"SPIDERX_REPO=\$SPIDERX_REPO; S2_BAG=\${S2_BAG:-<unset>}\"
  echo \"== direct tests (not collected by colcon) \$(date -u +%FT%TZ) at \$(git rev-parse HEAD), fresh installation \$F/install sourced\"
  echo '-- src/spiderx_scripts/test (pytest)'; (cd src/spiderx_scripts && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 timeout 600 python3 -m pytest -q -p no:cacheprovider test > \$D/direct_spiderx_scripts.log 2>&1; tail -1 \$D/direct_spiderx_scripts.log)
  echo '-- scripts/s2 (unittest)'; (cd scripts/s2 && PYTHONDONTWRITEBYTECODE=1 timeout 900 python3 -m unittest discover -v -s . -p 'test_*.py' > \$D/direct_s2.log 2>&1; tail -3 \$D/direct_s2.log)
  echo '-- scripts/s3 (unittest, all three test files)'; (cd scripts/s3 && PYTHONDONTWRITEBYTECODE=1 timeout 900 python3 -m unittest discover -v -s . -p 'test_*.py' > \$D/direct_s3.log 2>&1; tail -3 \$D/direct_s3.log)
  echo '-- scripts/s4 (unittest, S4a-S4d drivers and scorers)'; (cd scripts/s4 && PYTHONDONTWRITEBYTECODE=1 timeout 1500 python3 -m unittest discover -v -s . -p 'test_*.py' > \$D/direct_s4.log 2>&1; tail -3 \$D/direct_s4.log)" > $D/direct_tests.txt 2>&1
echo "end $(date -u +%FT%TZ)"
