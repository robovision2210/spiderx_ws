#!/usr/bin/env bash
# S4e harness template: idempotent stop of the processes THIS run owns (from the tested S4d-E
# cleanup). A process is stopped only through a pid, process group or session recorded in RUN;
# a name pattern alone never stops anything, so look-alike processes this run did not start are
# listed, not killed. Missing pid files (an interruption before the launch) are reported and
# skipped. The observer and its teleop belong to the operator's terminal, not to this run: the
# observer ends them itself (contract section 4.3).
# Owned: the launch (RUN/launch.pid; setsid: session = pid), the recorder (RUN/bag.pid; setsid:
# group = pid). Usage: cleanup.sh <run dir>
# Zombies are not running: a stopped process whose parent was interrupted first stays a zombie
# until it is reaped, and is neither waited for nor signalled (S4e template change).
RUN=$1
echo "[$(date -u +%T)] cleanup start"
rd() { [ -f "$RUN/$1" ] && tr -dc '0-9' < "$RUN/$1"; }
alive_in() {  # field 2 = process group, 3 = session; prints the live (non-zombie) members
  ps -e -o pid=,pgid=,sid=,stat= | awk -v f="$1" -v id="$2" '$f == id && $4 !~ /^Z/ {print $1}'
}
BP=$(rd bag.pid)
if [ -n "$BP" ] && [ -n "$(alive_in 2 "$BP")" ]; then
  kill -INT -- -"$BP" 2>/dev/null; for i in $(seq 1 30); do [ -z "$(alive_in 2 "$BP")" ] && break; sleep 0.5; done
  if [ -n "$(alive_in 2 "$BP")" ]; then kill -KILL -- -"$BP" 2>/dev/null; echo "recorder group $BP: SIGINT, then SIGKILL after 15 s"; else echo "recorder group $BP: stopped by SIGINT"; fi
else echo "recorder: ${BP:-no pid recorded}: not running"; fi
LP=$(rd launch.pid)
if [ -n "$LP" ] && [ -n "$(alive_in 3 "$LP")" ]; then
  kill -INT -- -"$LP" 2>/dev/null; for i in $(seq 1 40); do [ -z "$(alive_in 3 "$LP")" ] && break; sleep 0.5; done
  if [ -n "$(alive_in 3 "$LP")" ]; then echo "launch session $LP still has: $(alive_in 3 "$LP" | tr '\n' ' ')"; for q in $(alive_in 3 "$LP"); do kill -KILL "$q" 2>/dev/null; done; echo "launch session $LP: SIGKILL to those"
  else echo "launch session $LP: stopped by SIGINT"; fi
else echo "launch: ${LP:-no pid recorded}: not running"; fi
sleep 0.5
echo "[$(date -u +%T)] cleanup done; relevant processes still present (listed, not killed):"
pgrep -af "ign gazebo|gz sim|parameter_bridge|robot_state_publisher|ros2 launch|Xvfb|m55_|s4e_observer|ros2 bag|bag record" | grep -v -E "pgrep|cleanup.sh" || echo none
