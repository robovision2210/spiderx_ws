#!/usr/bin/env bash
# E11 validation (A), no motion: launch with the frozen stage scripts, wait for the controllers,
# then run scripts/m61_restore_and_verify_disabled.sh against the RUNNING launch. The script
# must stop it (launch group + virtual display), find no leftover, keep src/ equal to the base,
# rebuild cleanly and verify the disabled state in a fresh environment. Nothing is commanded.
# $1 = run dir, $2 = evidence dir for the script, $3 = build command (Cloud: setuptools 59 path)
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
step() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a $RUN/steps.log; }
step "start"; bash $EVT/stage_start.sh $RUN
step "wait controllers"; bash $EVT/stage_wait_controllers.sh $RUN || { step "CONTROLLERS NOT ACTIVE"; bash $EVT/stage_stop.sh $RUN; exit 1; }
step "restore and verify (the launch is still running)"
bash /home/user/spiderx_ws/scripts/m61_restore_and_verify_disabled.sh --base claude/stoic-shannon-ur2mes \
  --evidence "$2" --launch-pid-file $RUN/launch.pid --xvfb-pid-file $RUN/xvfb.pid --build-cmd "$3"
RC=$?
step "restore script exit=$RC"
tail -20 $RUN/launch.log > $RUN/launch_tail.txt
exit $RC
