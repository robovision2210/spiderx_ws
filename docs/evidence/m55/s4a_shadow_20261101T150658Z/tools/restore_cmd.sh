#!/usr/bin/env bash
# S4a shadow verification of the disabled state (the S3 shadow-2 restore_cmd.sh): nothing is
# enabled in this run, so --verify-only against the disabled baseline commit (no checkout, no
# rebuild, no enabling ref, no branch deletion), with the run's pid files and an explicit evidence
# directory. Exit 0 only for DISABLED AND VERIFIED.
R=/root/spiderx_evidence/s4a_shadow_20261101T150658Z
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base 7c1ecbc2654fb2bf80f23482812d8d89273cdc7e \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh --verify-only \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid
