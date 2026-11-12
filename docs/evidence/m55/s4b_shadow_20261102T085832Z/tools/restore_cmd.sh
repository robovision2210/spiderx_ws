#!/usr/bin/env bash
# S4b shadow verification of the disabled state (the S4a shadow restore_cmd.sh): nothing is
# enabled in this run, so --verify-only against the disabled baseline commit (no checkout, no
# rebuild, no enabling ref, no branch deletion), with the run's pid files and an explicit evidence
# directory. Exit 0 only for DISABLED AND VERIFIED.
R=/root/spiderx_evidence/s4b_shadow_20261102T085832Z
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base e406590a98ead5052b610316a5d23aba0c6edaf6 \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh --verify-only \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid
