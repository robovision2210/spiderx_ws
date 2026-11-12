#!/usr/bin/env bash
# S4d-M shadow verification of the disabled state (the S4c level-2 shadow restore_cmd.sh): nothing is
# enabled in this run, so --verify-only against the disabled baseline commit (no checkout, no
# rebuild, no enabling ref, no branch deletion), with the run's pid files and an explicit evidence
# directory. S4d change (OD12, M15): then the run installation (the fresh disabled installation this
# run sourced) is verified in a fresh environment (run_install_gates.sh). Exit 0 only for DISABLED AND
# VERIFIED and the run installation's three gates False.
R=/root/spiderx_evidence/s4d_shadow_20261110T072920Z
cd /home/user/spiderx_ws && env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash scripts/m61_restore_and_verify_disabled.sh --base 8f99bcc4ab27294265da91a31ce03999423acc7f \
  --evidence $R/restore --ros-setup /opt/mm/activate.sh --verify-only \
  --launch-pid-file $R/run/launch.pid --xvfb-pid-file $R/run/xvfb.pid
RC1=$?
bash $R/tools/run_install_gates.sh /root/spiderx_fresh_disabled_20261110T071506Z/install $R/run_install
RC2=$?
echo "restore --verify-only exit $RC1; run installation gates exit $RC2"
[ $RC1 = 0 ] && [ $RC2 = 0 ]
