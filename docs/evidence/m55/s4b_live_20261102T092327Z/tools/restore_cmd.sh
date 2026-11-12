#!/usr/bin/env bash
# S4b live restore (the S4a live restore_cmd.sh): the exact disabled baseline 2b22e44, the
# toolchain-only clean rebuild with st59site confined to the build, the run's pid files, and the
# enabling branch deleted only after every verification passed (enforced by the script).
R=/root/spiderx_evidence/s4b_live_20261102T092327Z; B=$(cat $R/enabling_branch.txt)
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base 2b22e443dd6abb1f80c61d8f52e670c9860e6a22 \
  --evidence $R/restore --enabling-ref "$B" --ros-setup /opt/mm/activate.sh \
  --build-cmd 'rm -rf build install && PYTHONPATH=/opt/mm/st59site colcon build --symlink-install' \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid --delete-enabling-branch
