#!/usr/bin/env bash
# One supervised launch-only observation run: launch, controllers active + 3 s, preflight,
# 120 s observation, read-only captures, /scan check, views, Ctrl+C-equivalent stop, analysis.
RUN=$(cd "$1" && pwd); EVT=$(dirname "$RUN")/tools
step() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a $RUN/steps.log; }
step "start"; bash $EVT/stage_start.sh $RUN
step "wait controllers"; bash $EVT/stage_wait_controllers.sh $RUN || { step "CONTROLLERS NOT ACTIVE"; bash $EVT/stage_stop.sh $RUN; exit 1; }
step "preflight"; bash $EVT/stage_observe.sh $RUN preflight
step "observe 120 s"; bash $EVT/stage_observe.sh $RUN observe
step "captures"; bash $EVT/stage_capture.sh $RUN
step "scan check"; (source $EVT/env.sh; python3 $EVT/scan_check.py $RUN/captures/capture.json \
  /home/user/spiderx_ws/src/spiderx_description/worlds/spiderx_fortress.sdf --mount 0 0 0.125 0 \
  --out $RUN/captures/scan_check.json > $RUN/captures/scan_check.txt 2>&1; echo "scan_check exit=$?" >> $RUN/captures/scan_check.txt
  python3 $EVT/scan_check.py $RUN/captures/capture.json \
  /home/user/spiderx_ws/src/spiderx_description/worlds/spiderx_fortress.sdf --mount 0 0 0.125 0 \
  --exclude-seams --out $RUN/captures/scan_check_seams_excluded.json > $RUN/captures/scan_check_seams_excluded.txt 2>&1
  python3 $EVT/scan_outliers.py $RUN/captures/capture.json \
  /home/user/spiderx_ws/src/spiderx_description/worlds/spiderx_fortress.sdf $RUN/captures/scan_outliers.json > $RUN/captures/scan_outliers.txt 2>&1)
step "views"; bash $EVT/stage_views.sh $RUN
step "stop"; bash $EVT/stage_stop.sh $RUN
step "analysis"; (source $EVT/env.sh; python3 $EVT/run_analysis.py $RUN | tee $RUN/analysis.txt)
step "end"
