#!/usr/bin/env bash
# M5.5 phase-1 shadow session in Cloud (gate False): launch, bag (no command can be sent),
# graph captures, scripted teleop, captures, clean stop.
R=$1; WS=/home/user/spiderx_ws
source /opt/mm/activate.sh >/dev/null 2>&1; source $WS/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 DISPLAY=:78; unset RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER
step() { echo "[$(date -u +%T)] $*" | tee -a $R/steps.log; }
mkdir -p $R/node_evidence $R/captures
Xvfb :78 -screen 0 1600x900x24 -nolisten tcp > $R/xvfb.log 2>&1 & echo $! > $R/xvfb.pid; sleep 2
step launch
setsid bash -c "source /opt/mm/activate.sh >/dev/null 2>&1; source $WS/install/setup.bash; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 DISPLAY=:78; exec ros2 launch spiderx_bringup fortress_m55_walking.launch.py evidence_dir:=$R/node_evidence" > $R/launch.log 2>&1 &
echo $! > $R/launch.pid
for i in $(seq 1 400); do grep -q "locomotion node up" $R/launch.log && break; grep -q "process has died" $R/launch.log && { step "LAUNCH FAILED"; break; }; sleep 1; done
step "node up after ${i} s: $(grep -o 'locomotion node up[^\"]*' $R/launch.log | head -1 | cut -c1-160)"
sleep 3
cap() { timeout 20 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $R/captures/$1_action_info.txt 2>&1
  for t in /leg_trajectory_controller/joint_trajectory /cmd_vel /spiderx/locomotion/status; do echo "== $t"; timeout 20 ros2 topic info -v $t; done > $R/captures/$1_topics.txt 2>&1
  timeout 20 ros2 node info /spiderx_locomotion > $R/captures/$1_node_info.txt 2>&1; }
step "captures before"; cap before
setsid ros2 bag record -s sqlite3 --include-hidden-topics -o $R/bag /spiderx/locomotion/status /joint_states \
  /spiderx/sim/world_poses /cmd_vel /leg_trajectory_controller/joint_trajectory /leg_trajectory_controller/controller_state \
  /leg_trajectory_controller/follow_joint_trajectory/_action/feedback > $R/bag_record.log 2>&1 &
BAG=$!; sleep 4
step "teleop driver"; python3 $R/tools/teleop_driver.py $R/teleop_driver.jsonl > $R/teleop_driver.out 2>&1; step "driver exit $?"
sleep 3; step "captures after"; cap after
kill -INT -- -$BAG 2>/dev/null || kill -INT $BAG; for i in $(seq 1 30); do pgrep -g $BAG >/dev/null || break; sleep 0.5; done
step stop; LP=$(cat $R/launch.pid); kill -INT -- -$LP 2>/dev/null
for i in $(seq 1 60); do pgrep -g $LP >/dev/null || break; sleep 1; done; echo "launch group empty after ${i} s" > $R/shutdown.txt
pgrep -g $LP -a >> $R/shutdown.txt 2>&1 && echo LEFTOVER >> $R/shutdown.txt
ros2 daemon stop >> $R/shutdown.txt 2>&1; kill -TERM $(cat $R/xvfb.pid) 2>/dev/null; sleep 2
echo "--- remaining:" >> $R/shutdown.txt; pgrep -af "ign gazebo|gz sim|ruby.*ign|parameter_bridge|controller_manager|robot_state_publisher|ros2 launch|_ros2_daemon|Xvfb|spawner|m55_" | grep -v pgrep >> $R/shutdown.txt || echo none >> $R/shutdown.txt
step end
