#!/usr/bin/env bash
# One supervised NO-MOTION launch for the E8 provenance measurement (not a criterion-6 run):
# launch (frozen stage scripts), controllers active + 3 s, the description, then a 15 s rosbag2
# recording of the pose stream with ALL entries, /joint_states, /clock and /robot_description,
# then the Ctrl+C-equivalent stop. Subscriptions only: nothing is commanded, no goal, no
# simulator control. The frozen analysis (spiderx_controller.m61a_link_check at 927acb2) runs
# afterwards, offline, on the bag.
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
step() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a $RUN/steps.log; }
step "start"; bash $EVT/stage_start.sh $RUN
step "wait controllers"; bash $EVT/stage_wait_controllers.sh $RUN || { step "CONTROLLERS NOT ACTIVE"; bash $EVT/stage_stop.sh $RUN; exit 1; }
source $EVT/env.sh
step "description"
python3 $EVT/capture_description.py --out $RUN/robot_description.urdf > $RUN/capture_description.txt 2>&1
echo "exit=$?" >> $RUN/capture_description.txt
step "record 15 s"
timeout -s INT 15 ros2 bag record -o $RUN/bag /spiderx/sim/world_poses /joint_states /clock /robot_description > $RUN/bag_record.log 2>&1
echo "record exit=$? (124 = stopped by the 15 s timeout, as intended)" >> $RUN/bag_record.log
step "stop"; bash $EVT/stage_stop.sh $RUN
step "end"
