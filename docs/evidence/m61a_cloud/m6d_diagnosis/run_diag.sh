#!/bin/bash
# One diagnostic invocation of the repository test under the observer plugin.
# usage: run_diag.sh LABEL STACK(original|repo) BURNERS
DG="$(cd "$(dirname "$0")" && pwd)"; LABEL=$1; STACK=$2; BURN=${3:-0}
WS=/home/user/spiderx_ws
OUT="$DG/runs/$LABEL"; mkdir -p "$OUT"
source /opt/mm/activate.sh >/dev/null 2>&1
source $WS/install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$DG:$PYTHONPATH"
export DIAG_OUT="$OUT" DIAG_STACK="$STACK" DIAG_TEST_DIR="$WS/src/spiderx_controller/test"
unset ROS_DOMAIN_ID
pids=()
for i in $(seq 1 "$BURN"); do python3 -c 'while True: pass' & pids+=($!); done
{ echo "label=$LABEL stack=$STACK burners=$BURN start=$(date -u +%FT%T.%3NZ) loadavg=$(cut -d' ' -f1-3 /proc/loadavg) nproc=$(nproc)"; } > "$OUT/meta.txt"
cd $WS/src/spiderx_controller
python3 -m pytest test/test_m6d_isolated_success.py -p diag_plugin -p no:cacheprovider \
  --basetemp="$OUT/basetemp" -q -rA > "$OUT/pytest.log" 2>&1
rc=$?
for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null
echo "rc=$rc end=$(date -u +%FT%T.%3NZ)" >> "$OUT/meta.txt"
echo "$LABEL rc=$rc"
