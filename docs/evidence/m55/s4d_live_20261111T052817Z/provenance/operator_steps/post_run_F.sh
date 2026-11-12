#!/usr/bin/env bash
# Operator steps after the guarded live run ended (F): leftovers, watchdog stand-down, bag hashes, restore.
R=/root/spiderx_evidence/s4d_live_20261111T052817Z; RUN=$R/run; J=/root/spiderx_evidence/s4d_live_journal.md
grep -q "guarded run exit" $RUN/run_wrapper.out || { echo "the guarded run has not ended"; exit 1; }
{ date -u +%FT%TZ; pgrep -af "ign gazebo|gz sim|parameter_bridge|robot_state_publisher|ros2 launch|Xvfb|spawner|m55_|s4d_driver|ros2 bag|bag record|strace|_ros2_daemon" | grep -v -E "pgrep|post_run_F" || echo none; } > $RUN/leftovers_after.txt
cat $RUN/leftovers_after.txt
touch $RUN/stopped.flag; WP=$(tr -dc '0-9' < $RUN/watchdog.pid)
for i in $(seq 1 30); do kill -0 $WP 2>/dev/null || break; sleep 1; done
if kill -0 $WP 2>/dev/null; then echo "$(date -u +%FT%TZ) watchdog $WP still running after stopped.flag" > $RUN/watchdog_standdown.txt
else echo "$(date -u +%FT%TZ) watchdog $WP stood down on stopped.flag without acting ($( [ -f $RUN/watchdog.txt ] && echo 'watchdog.txt EXISTS' || echo 'no watchdog.txt'))" > $RUN/watchdog_standdown.txt; fi
cat $RUN/watchdog_standdown.txt
B=$R/bag/bag_0.db3
{ echo "before restore and scoring $(date -u +%FT%TZ):"; sha256sum $B; stat -c '%s bytes' $B; } > $RUN/bag_hashes.txt; cat $RUN/bag_hashes.txt
cp $R/bag/metadata.yaml $RUN/bag_metadata.yaml 2>/dev/null
mkdir -p $R/restore
cd /home/user/spiderx_ws && { date -u +%FT%TZ; echo "branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD)"; git status --porcelain; echo "local/* branches:"; git branch --list 'local/*'; } > $R/restore/worktree_status_before_restore.txt
cat $R/restore/worktree_status_before_restore.txt
echo "- $(date -u +%FT%TZ) guarded run ended ($(tail -1 $RUN/run_wrapper.out)); leftovers: $(tail -n +2 $RUN/leftovers_after.txt | tr '\n' ';'); $(cat $RUN/watchdog_standdown.txt); bag $(head -2 $RUN/bag_hashes.txt | tail -1 | cut -c1-16)... $(tail -1 $RUN/bag_hashes.txt); restore starting" >> $J
bash $R/tools/restore_cmd.sh > $R/restore/restore_cmd.out 2>&1; RC=$?
echo "restore_cmd exit $RC"; tail -6 $R/restore/restore_cmd.out
echo "- $(date -u +%FT%TZ) restore_cmd exit $RC: $(grep -h -o 'DISABLED AND VERIFIED\|NOT VERIFIED[^\"]*' $R/restore/restore_verify_*.txt 2>/dev/null | head -1); $(tail -1 $R/restore/restore_cmd.out)" >> $J
exit $RC
