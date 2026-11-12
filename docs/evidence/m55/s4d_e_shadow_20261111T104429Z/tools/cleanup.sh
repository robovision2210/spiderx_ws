#!/usr/bin/env bash
# S4d-E shadow (from the S4d-M shadow cleanup.sh; only the driver name changed): idempotent stop of the processes THIS run owns (from the S4c level-2 shadow
# cleanup.sh). S4d change: a process is stopped only through a pid, process group or session that
# this run recorded in RUN, and only if its command line is the expected one; a name pattern alone
# never kills anything here. Owned:
#  - the driver: RUN/driver.pid (written by the run script just before exec), s4d_e_driver.py;
#  - each teleop the driver started: its pty session (pty.fork, so the session id is the pid the
#    driver logged as "teleop_started" in RUN/driver/driver.jsonl), m55_teleop_keyboard;
#  - the recorder: RUN/bag.pid (setsid: process group = pid), "bag record";
#  - strace: RUN/strace.pid;
#  - the launch: SIGINT to its process group through stage_stop.sh while the group has members
#    (which also stops the ros2 CLI daemon and Xvfb), then SIGKILL to members of the launch
#    session (setsid: session = RUN/launch.pid) that are still left;
#  - Xvfb: RUN/xvfb.pid.
# Runs from the guarded wrapper's EXIT/INT/TERM trap, and from the watchdog. Ends with a report of
# relevant processes still present; those are listed, not killed.
RUN=$1; EVT=$(dirname "$RUN")/tools
echo "[$(date -u +%T)] cleanup start"
rd() { [ -f "$RUN/$1" ] && tr -dc '0-9' < "$RUN/$1"; }
is() { [ -n "$1" ] && kill -0 "$1" 2>/dev/null && tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -q -- "$2"; }
DP=$(rd driver.pid)
if is "$DP" s4d_e_driver.py; then
  kill -INT "$DP"; for i in $(seq 1 20); do kill -0 "$DP" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "$DP" 2>/dev/null; then kill -KILL "$DP"; echo "driver $DP: SIGINT, then SIGKILL after 10 s"; else echo "driver $DP: stopped by SIGINT"; fi
else echo "driver: ${DP:-no pid recorded}: not running"; fi
TS=$(grep '"teleop_started"' "$RUN/driver/driver.jsonl" 2>/dev/null | grep -o '"pid": *[0-9]*' | grep -o '[0-9]*$' | tr '\n' ' ')
for T in $TS; do
  L=$(pgrep -a -s "$T" -f m55_teleop_keyboard 2>/dev/null)
  if [ -n "$L" ]; then pkill -KILL -s "$T" -f m55_teleop_keyboard; echo "teleop session $T: SIGKILL to: $(echo "$L" | tr '\n' ';')"
  else echo "teleop session $T: nothing left"; fi
done
[ -z "$TS" ] && echo "teleop sessions: none recorded"
BP=$(rd bag.pid)
if [ -n "$BP" ] && pgrep -g "$BP" -f "bag record" > /dev/null 2>&1; then
  kill -INT -- -"$BP" 2>/dev/null; for i in $(seq 1 30); do pgrep -g "$BP" > /dev/null || break; sleep 0.5; done
  if pgrep -g "$BP" > /dev/null; then kill -KILL -- -"$BP" 2>/dev/null; echo "recorder group $BP: SIGINT, then SIGKILL after 15 s"; else echo "recorder group $BP: stopped by SIGINT"; fi
else echo "recorder: ${BP:-no pid recorded}: not running"; fi
STP=$(rd strace.pid); is "$STP" strace && { kill -INT "$STP"; echo "strace $STP: SIGINT"; }
LP=$(rd launch.pid)
if [ -n "$LP" ] && pgrep -g "$LP" > /dev/null 2>&1; then bash "$EVT/stage_stop.sh" "$RUN" > /dev/null 2>&1; echo "launch group $LP: stage_stop.sh (SIGINT; shutdown.txt)"; fi
if [ -n "$LP" ] && pgrep -s "$LP" > /dev/null 2>&1; then
  echo "launch session $LP still has: $(pgrep -a -s "$LP" | tr '\n' ';')"; pkill -KILL -s "$LP"; echo "launch session $LP: SIGKILL to those"
fi
XP=$(rd xvfb.pid); is "$XP" Xvfb && { kill -TERM "$XP"; echo "Xvfb $XP: SIGTERM"; }
sleep 1
echo "[$(date -u +%T)] cleanup done; relevant processes still present (listed, not killed):"
pgrep -af "ign gazebo|gz sim|parameter_bridge|robot_state_publisher|ros2 launch|Xvfb|spawner|m55_|s[23]_driver|s4[abcd]_driver|ros2 bag|bag record|strace" | grep -v -E "pgrep|cleanup.sh" || echo none
