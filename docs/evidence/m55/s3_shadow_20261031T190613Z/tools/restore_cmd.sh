#!/usr/bin/env bash
# S3 shadow verification of the disabled state (adapted from the Phase C restore_cmd.sh): nothing was
# enabled in this run, so --verify-only against the disabled baseline commit (no checkout, no
# rebuild, no enabling ref, no branch deletion), with the run's pid files and an explicit evidence
# directory. Exit 0 only for DISABLED AND VERIFIED.
R=/root/spiderx_evidence/s3_shadow_20261031T190613Z
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base e0a952e7f0b0416e8b0276c24e98cbf914be99ae \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh --verify-only \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid
