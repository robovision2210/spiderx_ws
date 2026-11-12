#!/usr/bin/env bash
# Idempotent stop of everything an S2 run starts (driver, teleop, bag, node, launch, Xvfb).
# Runs from the run wrapper's EXIT/INT/TERM trap, so it also runs after an interruption.
RUN=$1; EVT=$(dirname "$RUN")/tools
echo "[$(date -u +%T)] cleanup start"
pkill -INT -f "s2_driver.py" 2>/dev/null; sleep 1; pkill -KILL -f "s2_driver.py" 2>/dev/null
pkill -KILL -f "m55_teleop_keyboard" 2>/dev/null
pkill -INT -f "ros2 bag record" 2>/dev/null; sleep 2; pkill -KILL -f "ros2 bag record" 2>/dev/null
if [ -f "$RUN/node.pid" ]; then NP=$(cat "$RUN/node.pid"); kill -INT -- -"$NP" 2>/dev/null; sleep 3; kill -KILL -- -"$NP" 2>/dev/null; fi
pkill -KILL -f "m55_locomotion_node" 2>/dev/null
if [ -f "$RUN/launch.pid" ] && pgrep -g "$(cat "$RUN/launch.pid")" >/dev/null 2>&1; then bash "$EVT/stage_stop.sh" "$RUN" >/dev/null 2>&1; fi
[ -f "$RUN/xvfb.pid" ] && kill -TERM "$(cat "$RUN/xvfb.pid")" 2>/dev/null
echo "[$(date -u +%T)] cleanup done; remaining:"
pgrep -af "ign gazebo|gz sim|parameter_bridge|robot_state_publisher|ros2 launch|Xvfb|spawner|m55_|s2_driver|ros2 bag" | grep -v -E "pgrep|cleanup.sh" || echo none
