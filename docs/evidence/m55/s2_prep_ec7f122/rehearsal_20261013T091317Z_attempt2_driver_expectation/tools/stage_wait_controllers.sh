#!/usr/bin/env bash
# Terminal B: wait until both documented controllers are active, then the documented 3 s.
RUN=$1
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
{ echo "shell B: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY DISPLAY=$DISPLAY";
  python -c "import rclpy; print('rmw', rclpy.get_rmw_implementation_identifier())"; } > $RUN/env_B.txt
t0=$(date +%s)
while true; do
  out=$(timeout 15 ros2 control list_controllers 2>&1 | sed 's/\x1b\[[0-9;]*m//g')
  if grep -q "rclpy.ok()" <<< "$out"; then        # broken CLI daemon: restart it, then retry
    echo "$(date -u +%T) ros2 CLI daemon fault; restarting it" >> $RUN/daemon_events.txt
    ros2 daemon stop >> $RUN/daemon_events.txt 2>&1; ros2 daemon start >> $RUN/daemon_events.txt 2>&1
  fi
  n=$(grep -cE "^(joint_state_broadcaster|leg_trajectory_controller) .*[[:space:]]active$" <<< "$out")
  if [ "$n" = 2 ]; then break; fi
  if grep -qE "\[ERROR\] \[create-[0-9]+\]: process has died|startup failed" $RUN/launch.log; then
    echo "LAUNCH FAILED before the controllers came up (see launch.log)"
    { date -u +%T; ps -eo pid,etime,stat,cmd | grep -E "ign gazebo|ruby|gz sim" | grep -v grep;
      timeout 10 ign service -l; echo "--- topics"; timeout 10 ign topic -l;
      echo "--- /gazebo/starting_world"; timeout 10 ign topic -i -t /gazebo/starting_world;
      ls -t ~/.ignition/gazebo/log | head -3; } > $RUN/launch_failure_diagnostics.txt 2>&1
    cp ~/.ignition/auto_default.log $RUN/gazebo_gui_auto_default.log 2>/dev/null
    exit 2
  fi
  if [ $(( $(date +%s) - t0 )) -gt 300 ]; then echo "TIMEOUT waiting for controllers"; echo "$out"; exit 1; fi
  sleep 3
done
echo "$out" > $RUN/controllers_active.txt
date -u +%Y-%m-%dT%H:%M:%S.%NZ > $RUN/controllers_active_utc.txt
echo "both controllers active after $(( $(date +%s) - t0 )) s of polling"; cat $RUN/controllers_active.txt
sleep 3
