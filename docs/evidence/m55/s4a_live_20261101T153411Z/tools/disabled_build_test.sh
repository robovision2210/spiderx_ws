#!/usr/bin/env bash
# One clean disabled build and full suite (Phase C step 2 procedure): toolchain only (env -i),
# st59site confined to the build, PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 for the tests, then the
# fresh-collection check. Usage: disabled_build_test.sh <run root>
R=$1; D=$R/disabled; WS=/home/user/spiderx_ws
cd $WS
echo "start $(date -u +%FT%TZ) branch $(git rev-parse --abbrev-ref HEAD) head $(git rev-parse HEAD) worktree $(git status --porcelain | wc -l)"
T0=$(date +%s); echo $T0 > $D/t0.txt
env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin R=$R T0=$T0 bash -c '
  D=$R/disabled; source /opt/mm/activate.sh >/dev/null 2>&1; cd /home/user/spiderx_ws
  rm -rf build install && PYTHONPATH=/opt/mm/st59site colcon build --symlink-install > $D/build.log 2>&1; echo "build exit=$?"
  source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --event-handlers console_direct- > $D/test.log 2>&1; echo "test exit=$?"
  colcon test-result --verbose > $D/colcon_test_result.txt 2>&1; echo "result exit=$?"
  tail -1 $D/colcon_test_result.txt
  python3 $R/tools/collection_check.py /home/user/spiderx_ws $T0 $D/colcon_test_result.txt > $D/collection_check.txt 2>&1
  tail -2 $D/collection_check.txt'
echo "end $(date -u +%FT%TZ)"
