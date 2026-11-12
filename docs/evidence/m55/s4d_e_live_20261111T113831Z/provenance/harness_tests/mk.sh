#!/usr/bin/env bash
# mk.sh <case> <enabled|disabled|real> [enabling start age s]: a case sandbox for the E live run script. "real" keeps the
# real env.sh and paths (only the launch, recorder and captures are stubbed); otherwise the fake installation, the
# scratch repo and its base. The enabling start epoch is now minus the age (default 60 s).
T=$(dirname "$(realpath "$0")"); R=/root/spiderx_evidence/s4d_e_live_20261111T113831Z; C=$T/$1; K=$2; BASE=$(cat $T/base.txt)
mkdir -p $C/tools $C/run
cp -p $R/tools/cleanup.sh $R/tools/watchdog.sh $R/tools/s4d_e_live_guarded.sh $C/tools/
echo $(( $(date +%s) - ${3:-60} )) > $C/enabling_start_epoch.txt
SUB=(-e "s#setsid ros2 bag record#setsid bash $T/stub/rec.sh#" -e 's#^cap() { #cap() { echo "STUB-CAP $1"; return 0; #')
if [ "$K" = real ]; then cp -p $R/tools/env.sh $C/tools/env.sh
else
  SUB+=(-e "s#^RUNINST=/root/spiderx_e_live_enabled_20261111T113831Z/install#RUNINST=$T/fake_$K/install#" -e "s#^BASE=16d939f5a0e06afa3478093f0b491e3e8ce60673#BASE=$BASE#" -e "s#WS=/home/user/spiderx_ws#WS=$T/repo#")
  printf 'source /opt/mm/activate.sh >/dev/null 2>&1\nsource /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash\nsource %s/fake_%s/install/setup.bash\nexport ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 DISPLAY=:77\nunset RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER\n' $T $K > $C/tools/env.sh
fi
sed "${SUB[@]}" $R/tools/s4d_e_live_run.sh > $C/tools/s4d_e_live_run.sh
cat > $C/tools/stage_start.sh <<'S'
#!/usr/bin/env bash
RUN=$1; setsid bash -c 'exec -a "STUB-LAUNCH-dummy" sleep 300' > /dev/null 2>&1 & echo $! > $RUN/launch.pid
bash -c 'exec -a "STUB-XVFB-dummy" sleep 300' > /dev/null 2>&1 & echo $! > $RUN/xvfb.pid
echo "[INFO] [m55_locomotion_node-7]: STUB locomotion node up in DISPATCH mode (dispatch gate True)" > $RUN/launch.log
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
diff $R/tools/s4d_e_live_run.sh $C/tools/s4d_e_live_run.sh > $C/run_script_test_copy.diff; true
