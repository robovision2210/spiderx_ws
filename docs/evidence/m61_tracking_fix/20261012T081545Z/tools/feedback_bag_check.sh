#!/usr/bin/env bash
# READ-ONLY check of the plan's E-step bag change (Cloud report §14.7), on the DISABLED build,
# with no goal: record the hidden action FEEDBACK topic (never _action/status, whose subscribers
# count as competing FollowJointTrajectory clients), then run the production readiness rehearsal
# (no session, no goal; send and cancel raise). Expected: the bag subscribes to _action/feedback,
# and readiness is READY with 0 foreign action clients.
#   feedback_bag_check.sh <run dir>      (a launch from the frozen harness must be up)
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
T=$RUN/feedback_bag_check; mkdir -p $T
FB=/leg_trajectory_controller/follow_joint_trajectory/_action/feedback
setsid ros2 bag record -s sqlite3 --include-hidden-topics -o $T/bag /joint_states /clock \
  /leg_trajectory_controller/controller_state $FB > $T/bag_record.log 2>&1 &
BAG=$!
sleep 5
ros2 topic info -v $FB > $T/feedback_topic_info.txt 2>&1
ros2 topic info -v /leg_trajectory_controller/follow_joint_trajectory/_action/status \
  > $T/status_topic_info.txt 2>&1
date -u +%FT%T.%NZ > $T/rehearsal_start_utc.txt
PYTHONUNBUFFERED=1 ros2 run spiderx_controller m61_readiness_rehearsal --domain-id 0 \
  --out $T/rehearsal > $T/rehearsal.txt 2>&1
echo "exit=$?" >> $T/rehearsal.txt
kill -INT -- -$BAG 2>/dev/null
for i in $(seq 1 30); do pgrep -g $BAG > /dev/null || break; sleep 0.5; done
cp $T/bag/metadata.yaml $T/bag_metadata.yaml 2>/dev/null; rm -rf $T/bag
echo "bag subscribed to feedback: $(grep -c "Subscribed to topic '$FB'" $T/bag_record.log)"
echo "rehearsal: $(grep -h '^READY\|^NOT READY\|exit=' $T/rehearsal.txt | tr '\n' ' ')"
echo "status subscribers: $(grep -c 'Endpoint type: SUBSCRIPTION' $T/status_topic_info.txt)"
