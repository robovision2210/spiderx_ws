#!/usr/bin/env bash
# Terminal B, after the observation: read-only graph/controller/Gazebo/scan/visual captures.
RUN=$1
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
C=$RUN/captures; mkdir -p $C
timeout 30 ros2 control list_controllers > $C/controllers.txt 2>&1
timeout 30 ros2 control list_hardware_interfaces > $C/hardware_interfaces.txt 2>&1
timeout 30 ros2 node list > $C/nodes.txt 2>&1
timeout 30 ros2 topic list -t > $C/topics.txt 2>&1
timeout 30 ros2 topic info -v /leg_trajectory_controller/joint_trajectory > $C/command_topic_info.txt 2>&1
timeout 30 ros2 action info /leg_trajectory_controller/follow_joint_trajectory > $C/action_info.txt 2>&1
timeout 60 ros2 topic echo /scan --once > $C/scan_once.yaml 2>&1
timeout 30 ign model -m spiderx -p > $C/ign_model_pose.txt 2>&1
timeout 30 ign model -m spiderx -l dummy_link > $C/ign_link_dummy_link.txt 2>&1
timeout 30 ign model -m spiderx -j spiderx_fixed_base_weld > $C/ign_joint_weld.txt 2>&1
python3 $EVT/capture.py --domain-id 0 --seconds 10 --scans 40 --out $C/capture.json > $C/capture.txt 2>&1
echo "capture exit=$?" >> $C/capture.txt
import -window root -display :99 $C/screenshot_gui.png > $C/screenshot.txt 2>&1
echo "screenshot exit=$?" >> $C/screenshot.txt
cat $C/capture.txt $C/screenshot.txt
