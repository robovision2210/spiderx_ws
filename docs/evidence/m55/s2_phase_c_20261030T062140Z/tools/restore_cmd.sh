#!/usr/bin/env bash
# Phase C restore (S2 design §2 F; Phase C draft §5): the exact base branch, the toolchain-only clean
# rebuild with st59site confined to the build, the run's pid files, and branch deletion only after
# every verification passed (enforced by the script).
R=/root/spiderx_evidence/s2_phase_c_20261030T062140Z; B=$(cat $R/enabling_branch.txt)
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base claude/spiderx-sim-actuator-model \
  --evidence $R/restore --enabling-ref "$B" --ros-setup /opt/mm/activate.sh \
  --build-cmd 'rm -rf build install && PYTHONPATH=/opt/mm/st59site colcon build --symlink-install' \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid --delete-enabling-branch
