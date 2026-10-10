#!/usr/bin/env bash
# Terminal B: the documented read-only preflight, then the 120 s (wall) observation.
RUN=$1; MODE=$2
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
if [ "$MODE" = preflight ]; then
  ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --preflight --out $RUN/observer > $RUN/preflight.txt 2>&1
  echo "preflight exit=$?" >> $RUN/preflight.txt; cat $RUN/preflight.txt
else
  ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --duration 120 --out $RUN/observer > $RUN/observe.txt 2>&1
  echo "observe exit=$?" >> $RUN/observe.txt; cat $RUN/observe.txt
fi
