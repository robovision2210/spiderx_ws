#!/usr/bin/env bash
# mk.sh <case> <disabled|enabled|missing>: an E shadow harness sandbox: the E run script (test copy:
# only the run installation path, the workspace, the recorder and the captures substituted), the live
# cleanup.sh, watchdog.sh and guarded wrapper (byte-identical copies), stub stages and restore.
T=$(dirname "$(realpath "$0")"); R=$(cat $T/root.txt); C=$T/$1; K=$2
mkdir -p $C/tools $C/run
cp -p $R/tools/cleanup.sh $R/tools/watchdog.sh $R/tools/s4d_e_shadow_guarded.sh $C/tools/
INST=$T/fake_$K/install; [ "$K" = missing ] && INST=$T/fake_disabled/install
RUNINST=$INST; [ "$K" = missing ] && RUNINST=$T/nonexistent/install
SUB=(-e "s#setsid ros2 bag record#setsid bash $T/stub/rec.sh#" -e 's#^cap() { #cap() { echo "STUB-CAP $1"; return 0; #'
     -e "s#^RUNINST=/root/spiderx_fresh_disabled_20261111T104338Z/install#RUNINST=$RUNINST#" -e "s#WS=/home/user/spiderx_ws#WS=$T/repo#")
printf 'source /opt/mm/activate.sh >/dev/null 2>&1\nsource /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash\nsource %s/setup.bash\nexport ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 DISPLAY=:77\nunset RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER\n' $INST > $C/tools/env.sh
sed "${SUB[@]}" $R/tools/s4d_e_shadow_run.sh > $C/tools/s4d_e_shadow_run.sh
cat > $C/tools/stage_start.sh <<'S'
#!/usr/bin/env bash
RUN=$1; setsid bash -c 'exec -a "STUB-LAUNCH-dummy" sleep 300' > /dev/null 2>&1 & echo $! > $RUN/launch.pid
bash -c 'exec -a "STUB-XVFB-dummy" sleep 300' > /dev/null 2>&1 & echo $! > $RUN/xvfb.pid
echo "[INFO] [m55_locomotion_node-7]: STUB locomotion node up in SHADOW mode (dispatch gate False)" > $RUN/launch.log
echo "STUB launch pid $(cat $RUN/launch.pid)"
S
printf '#!/usr/bin/env bash\necho "STUB controllers active"; exit 0\n' > $C/tools/stage_wait_controllers.sh
cat > $C/tools/stage_stop.sh <<'S'
#!/usr/bin/env bash
RUN=$1; LP=$(cat $RUN/launch.pid 2>/dev/null); [ -n "$LP" ] && kill -KILL -- -$LP 2>/dev/null; XP=$(cat $RUN/xvfb.pid 2>/dev/null); [ -n "$XP" ] && kill -KILL $XP 2>/dev/null
echo "STUB stage_stop: launch group $LP and xvfb $XP stopped" >> $RUN/shutdown.txt
S
printf '#!/usr/bin/env bash\necho "STUB restore_cmd.sh called $(date -u +%%T)"\n' > $C/tools/restore_cmd.sh
chmod +x $C/tools/*.sh
diff $R/tools/s4d_e_shadow_run.sh $C/tools/s4d_e_shadow_run.sh > $C/run_script_test_copy.diff; true
