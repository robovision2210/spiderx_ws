#!/usr/bin/env bash
# Ctrl+C equivalent (SIGINT to the launch's process group), daemon stop, display stop, cleanup check.
RUN=$1
EVT=$(dirname "$RUN")/tools
source $EVT/env.sh
LP=$(cat $RUN/launch.pid)
date -u +%Y-%m-%dT%H:%M:%S.%NZ > $RUN/stop_utc.txt
kill -INT -- -$LP 2>/dev/null; echo "SIGINT sent to process group $LP" > $RUN/shutdown.txt
for i in $(seq 1 60); do
  if ! pgrep -g $LP > /dev/null; then echo "process group empty after ${i} s" >> $RUN/shutdown.txt; break; fi
  sleep 1
done
pgrep -g $LP -a >> $RUN/shutdown.txt 2>&1 && echo "LEFTOVER processes in the launch group above" >> $RUN/shutdown.txt
ros2 daemon stop >> $RUN/shutdown.txt 2>&1
XP=$(cat $RUN/xvfb.pid); kill -TERM $XP 2>/dev/null
for i in $(seq 1 20); do ps -p $XP > /dev/null 2>&1 || break; sleep 0.5; done   # exited AND reaped
ps -p $XP -o pid=,stat=,cmd= >> $RUN/shutdown.txt 2>&1 && echo "Xvfb $XP still present after 10 s" >> $RUN/shutdown.txt
sleep 1
echo "--- remaining (expect nothing):" >> $RUN/shutdown.txt
pgrep -af "ign gazebo|gz sim|ruby.*ign|parameter_bridge|controller_manager|robot_state_publisher|ros2 launch|_ros2_daemon|Xvfb|spawner|create -name" >> $RUN/shutdown.txt || echo "none" >> $RUN/shutdown.txt
tail -20 $RUN/launch.log > $RUN/launch_tail.txt
cat $RUN/shutdown.txt
