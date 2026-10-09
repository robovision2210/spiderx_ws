#!/usr/bin/env bash
# Terminal B: wait until both documented controllers are active, then the documented 3 s.
RUN=$1
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
{ echo "shell B: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY DISPLAY=$DISPLAY";
  python -c "import rclpy; print('rmw', rclpy.get_rmw_implementation_identifier())"; } > $RUN/env_B.txt
t0=$(date +%s)
while true; do
  out=$(timeout 15 ros2 control list_controllers 2>&1)
  n=$(grep -cE "^(joint_state_broadcaster|leg_trajectory_controller).* active" <<< "$out")
  if [ "$n" = 2 ]; then break; fi
  if [ $(( $(date +%s) - t0 )) -gt 300 ]; then echo "TIMEOUT waiting for controllers"; echo "$out"; exit 1; fi
  sleep 3
done
echo "$out" > $RUN/controllers_active.txt
date -u +%Y-%m-%dT%H:%M:%S.%NZ > $RUN/controllers_active_utc.txt
echo "both controllers active after $(( $(date +%s) - t0 )) s of polling"; cat $RUN/controllers_active.txt
sleep 3
