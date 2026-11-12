#!/usr/bin/env bash
# S4c level-1 live restore (the S4b live restore_cmd.sh): the exact disabled baseline ea69d3f, the
# toolchain-only rebuild with st59site confined to the build, the run's pid files, and the
# enabling branch deleted only after every verification passed (enforced by the script).
# Changed before the launch: the rebuild is INCREMENTAL (no deletion of build/ and install/). A
# built-in safety check stopped the clean-build directory deletion in this session; that deletion
# is left to the owner. The enabling diff is two Python lines, installed as symlinks into src/.
R=/root/spiderx_evidence/s4c_l1_live_20261102T182303Z; B=$(cat $R/enabling_branch.txt)
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base ea69d3fbff36d210d0bcf9a3e8e152e84f162782 \
  --evidence $R/restore --enabling-ref "$B" --ros-setup /opt/mm/activate.sh \
  --build-cmd 'PYTHONPATH=/opt/mm/st59site colcon build --symlink-install' \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid --delete-enabling-branch
