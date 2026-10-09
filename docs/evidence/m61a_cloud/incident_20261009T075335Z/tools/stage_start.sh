#!/usr/bin/env bash
# Start the virtual display and the launch (terminal A) for run directory $1.
RUN=$1
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
{ echo "shell A: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY DISPLAY=$DISPLAY";
  python -c "import rclpy; print('rmw', rclpy.get_rmw_implementation_identifier())";
  echo "AMENT_PREFIX_PATH head: ${AMENT_PREFIX_PATH%%:*}"; } > $RUN/env_A.txt
Xvfb :99 -screen 0 1600x900x24 -nolisten tcp > $RUN/xvfb.log 2>&1 &
echo $! > $RUN/xvfb.pid
sleep 2
date -u +%Y-%m-%dT%H:%M:%S.%NZ > $RUN/launch_start_utc.txt
# own session/process group, so SIGINT to the group = Ctrl+C in a foreground terminal
setsid bash -c "source $EVT/env.sh; exec ros2 launch spiderx_bringup fortress_m61a_fixed_base.launch.py" > $RUN/launch.log 2>&1 &
echo $! > $RUN/launch.pid
sleep 1
echo "launch pid $(cat $RUN/launch.pid) pgid $(ps -o pgid= -p $(cat $RUN/launch.pid) | tr -d ' ') xvfb $(cat $RUN/xvfb.pid)"
