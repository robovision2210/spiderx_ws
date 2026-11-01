#!/usr/bin/env bash
# S3 live restore (the Phase C restore_cmd.sh): the exact disabled baseline d918799, the
# toolchain-only clean rebuild with st59site confined to the build, the run's pid files, and the
# enabling branch deleted only after every verification passed (enforced by the script).
R=/root/spiderx_evidence/s3_free_base_20261101T115002Z; B=$(cat $R/enabling_branch.txt)
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base d91879989138543f43b6c6733217669b773c0337 \
  --evidence $R/restore --enabling-ref "$B" --ros-setup /opt/mm/activate.sh \
  --build-cmd 'rm -rf build install && PYTHONPATH=/opt/mm/st59site colcon build --symlink-install' \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid --delete-enabling-branch
