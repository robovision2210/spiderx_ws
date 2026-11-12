#!/usr/bin/env bash
# S4c level-1 follow-up (offline): ONE fresh DISABLED build and full suite with NEW colcon bases.
# The bases are new, uniquely named and previously nonexistent, outside the tracked tree; the
# existing build/, install/ and log/ are not touched and nothing is deleted. Each step runs in a
# fresh shell (env -i): the documented toolchain, then the verified r2 overlay local_setup. The
# existing SpiderX installation is never sourced, and any of its paths in the environment refuses
# the step. st59site is on PYTHONPATH for the build command only; tests use
# PYTEST_DISABLE_PLUGIN_AUTOLOAD=1. Usage: fresh_build_test.sh <follow-up root> <fresh base dir>
FU=$1; F=$2; WS=/home/user/spiderx_ws; D=$FU/fresh
[ -e "$F" ] && { echo "REFUSED: $F exists (bases must be new)"; exit 2; }
mkdir -p "$F" "$D"
fresh() { env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  F="$F" WS="$WS" D="$D" FU="$FU" bash --noprofile --norc -c "source /opt/mm/activate.sh >/dev/null 2>&1; source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash; $1"; }
envdump='for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do echo "$v=${!v}"; done'
cd $WS
echo "start $(date -u +%FT%TZ) branch $(git rev-parse --abbrev-ref HEAD) head $(git rev-parse HEAD) worktree $(git status --porcelain | wc -l)"
echo "bases: build $F/build install $F/install log $F/log (created now; previously nonexistent)"
date +%s > $D/t0.txt
fresh "$envdump > \$D/env_build.txt
  if grep -q '/home/user/spiderx_ws/install' \$D/env_build.txt; then echo 'REFUSED: the existing installation is in the build environment'; exit 2; fi
  cd \$WS && PYTHONPATH=/opt/mm/st59site colcon --log-base \$F/log build --build-base \$F/build --install-base \$F/install --symlink-install > \$D/build.log 2>&1; echo \"build exit=\$?\"" || exit 2
fresh "source \$F/install/setup.bash; $envdump > \$D/env_test.txt
  if grep -q '/home/user/spiderx_ws/install' \$D/env_test.txt; then echo 'REFUSED: the existing installation is in the test environment'; exit 2; fi
  cd \$WS && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon --log-base \$F/log test --build-base \$F/build --install-base \$F/install --event-handlers console_direct- > \$D/test.log 2>&1; echo \"test exit=\$?\"
  colcon --log-base \$F/log test-result --test-result-base \$F/build --verbose > \$D/colcon_test_result.txt 2>&1; echo \"result exit=\$?\"
  tail -1 \$D/colcon_test_result.txt
  python3 \$FU/tools/collection_check.py \$F \$(cat \$D/t0.txt) \$D/colcon_test_result.txt > \$D/collection_check.txt 2>&1
  tail -2 \$D/collection_check.txt" || exit 2
fresh "source \$F/install/setup.bash; cd \$WS
  echo \"== direct tests (not collected by colcon) \$(date -u +%FT%TZ) at \$(git rev-parse HEAD), fresh installation \$F/install sourced\"
  echo '-- src/spiderx_scripts/test (pytest)'; (cd src/spiderx_scripts && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 timeout 600 python3 -m pytest -q -p no:cacheprovider test 2>&1 | tail -1)
  echo '-- scripts/s3 (unittest, all three test files)'; (cd scripts/s3 && PYTHONDONTWRITEBYTECODE=1 timeout 900 python3 -m unittest discover -s . -p 'test_*.py' 2>&1 | tail -3)
  echo '-- scripts/s4 (unittest, S4a, S4b and S4c drivers and scorers)'; (cd scripts/s4 && PYTHONDONTWRITEBYTECODE=1 timeout 1500 python3 -m unittest discover -s . -p 'test_*.py' 2>&1 | tail -3)" > $D/direct_tests.txt 2>&1
echo "end $(date -u +%FT%TZ)"
