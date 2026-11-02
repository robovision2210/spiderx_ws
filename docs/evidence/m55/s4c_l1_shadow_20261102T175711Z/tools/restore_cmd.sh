#!/usr/bin/env bash
# S4c level-1 shadow verification of the disabled state (the S4b shadow restore_cmd.sh): nothing is
# enabled in this run, so --verify-only against the disabled baseline commit (no checkout, no
# rebuild, no enabling ref, no branch deletion), with the run's pid files and an explicit evidence
# directory. Exit 0 only for DISABLED AND VERIFIED.
R=/root/spiderx_evidence/s4c_l1_shadow_20261102T175711Z
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base 069385c8e49008b2b6f1c66d12c105f2d75923de \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh --verify-only \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid
