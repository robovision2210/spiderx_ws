#!/usr/bin/env bash
# S4c level-2 live restore (the S4c level-1 live restore_cmd.sh): the exact disabled baseline 77d97fc,
# the toolchain-only INCREMENTAL rebuild (colcon build --symlink-install, st59site confined to the
# build; the owner authorized this established procedure for this attempt; no deletion of build/,
# install/ or log/), the run's pid files, and the enabling branch deleted only after every
# verification passed (enforced by the script). Fixed before enabling; not changed after the launch.
# The enabling diff is two Python lines, installed as symlinks into src/.
R=/root/spiderx_evidence/s4c_l2_live_20261103T111733Z; B=$(cat $R/enabling_branch.txt)
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base 77d97fc765a54dcc95f2f8857b70511e4454ea70 \
  --evidence $R/restore --enabling-ref "$B" --ros-setup /opt/mm/activate.sh \
  --build-cmd 'PYTHONPATH=/opt/mm/st59site colcon build --symlink-install' \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid --delete-enabling-branch
