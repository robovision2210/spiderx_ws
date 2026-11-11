#!/usr/bin/env bash
# S4d-E shadow verification of the disabled state (the S4d-M shadow restore_cmd.sh): nothing is enabled
# in this run, so --verify-only against the E branch commit that was validated (no checkout, no
# rebuild, no enabling ref, no branch deletion), with the run's pid files (passed only if they exist,
# as in the tested S4d-M live restore_cmd.sh: before a launch there is nothing to stop) and an
# explicit evidence directory; then the run installation (the fresh disabled E installation this
# run sourced) in a fresh environment (run_install_gates.sh). Exit 0 only for DISABLED AND VERIFIED
# and the run installation's three gates False.
R=/root/spiderx_evidence/s4d_e_shadow_20261111T104429Z
ARGS=(); [ -f $R/run/launch.pid ] && ARGS+=(--launch-pid-file $R/run/launch.pid)
[ -f $R/run/xvfb.pid ] && ARGS+=(--xvfb-pid-file $R/run/xvfb.pid)
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base 87198b951506b35ee4d95f3c124c0327a9647ae6 \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh --verify-only "${ARGS[@]}"
RC1=$?
bash $R/tools/run_install_gates.sh /root/spiderx_fresh_disabled_20261111T104338Z/install $R/run_install
RC2=$?
echo "restore --verify-only exit $RC1; run installation gates exit $RC2"
[ $RC1 = 0 ] && [ $RC2 = 0 ]
