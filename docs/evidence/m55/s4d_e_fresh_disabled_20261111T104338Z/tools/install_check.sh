#!/usr/bin/env bash
# S4c follow-up: one installation checked in a fresh shell (env -i): the documented toolchain, the
# verified r2 overlay local_setup, then ONLY this installation's setup.bash; run from /. Records the
# environment, the package prefix and module paths, the three gates, the library and config
# (install_check.py), and ONLY IF the gates are False and imported from this installation, both
# documented M6 --live refusals (the restore script's invocation: unused domain 231, localhost
# only). No simulator, graph preflight or goal.
# Usage: install_check.sh <label> <install prefix> <other installation prefix> <out dir>
LABEL=$1; P=$2; OTHER=$3; O=$4; FU=$(dirname "$(dirname "$(realpath "$0")")")
BASE_LIB=/home/user/spiderx_ws/docs/evidence/m55/s4c_l1_live_20261102T182303Z/disabled/library.json
mkdir -p "$O"
env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 P="$P" OTHER="$OTHER" O="$O" FU="$FU" BASE_LIB="$BASE_LIB" \
  LABEL="$LABEL" bash --noprofile --norc -c '
source /opt/mm/activate.sh >/dev/null 2>&1
source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash
source $P/setup.bash
cd /
F=""
echo "== installation $LABEL: $P ($(date -u +%FT%TZ))"
for v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; do echo "$v=${!v}"; done > $O/env.txt
if grep -q "$OTHER" $O/env.txt; then echo "FAIL the other installation ($OTHER) is in this environment"; F="$F env"
else echo "PASS the other installation ($OTHER) is not in this environment"; fi
PFX=$(ros2 pkg prefix spiderx_controller 2>&1 | tail -1); echo "ros2 pkg prefix spiderx_controller: $PFX"
if [ "$(realpath -m "$PFX")" = "$(realpath -m "$P/spiderx_controller")" ]; then echo "PASS the package prefix is this installation"
else echo "FAIL package prefix"; F="$F prefix"; fi
PYTHONDONTWRITEBYTECODE=1 python3 $FU/tools/install_check.py $P $BASE_LIB $O > $O/install_check_py.txt 2>&1; RC=$?
PYTHONDONTWRITEBYTECODE=1 python3 -c "
import json
d = json.load(open(\"$O/imports_gates_library.json\"))
for k, v in d[\"modules\"].items():
    print(\"module\", k, v[\"imported_as\"], \"->\", v[\"resolves_to\"], \"(this installation:\", v[\"imported_from_this_installation\"], \")\")
print(\"gates\", d[\"gates\"]); print(\"library\", d[\"library\"]); print(\"config\", d[\"config\"])" 2>&1
if [ $RC = 0 ]; then
  echo "PASS all three gates False, every module imported from this installation"
  for TOOL in m6_gait_replay.py m6_live_playback; do
    X=$P/spiderx_controller/lib/spiderx_controller/$TOOL; echo "executable $TOOL: $X -> $(readlink -f $X)"
    OUT=$(timeout 120 ros2 run spiderx_controller $TOOL --live --domain-id 231 < /dev/null 2>&1); LRC=$?
    echo "$OUT" > $O/refusal_$TOOL.txt
    if [ $LRC = 3 ] && grep -q "HARD-DISABLED" <<< "$OUT"; then echo "PASS $TOOL --live refused, exit 3, HARD-DISABLED"
    else echo "FAIL $TOOL --live exit $LRC"; F="$F refusal_$TOOL"; fi
  done
else
  echo "FAIL gates or module paths (install_check.py exit $RC): --live NOT run"; F="$F gates"
fi
grep -q "\"library_json_equals_baseline\": true" $O/imports_gates_library.json && echo "PASS library.json = the disabled baseline" || { echo "FAIL library.json"; F="$F library"; }
grep -q "\"equals_src\": true" $O/imports_gates_library.json && echo "PASS the loaded config = src" || { echo "FAIL config"; F="$F config"; }
[ -z "$F" ] && echo "INSTALLATION $LABEL VERDICT: PASS" || echo "INSTALLATION $LABEL VERDICT: FAIL:$F"'
