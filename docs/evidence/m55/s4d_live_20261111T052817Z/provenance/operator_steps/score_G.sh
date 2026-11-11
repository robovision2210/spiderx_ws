#!/usr/bin/env bash
# G: the unchanged committed scorer WITHOUT --shadow, after restoration; bag hash before/after; supplementary analysis.
R=/root/spiderx_evidence/s4d_live_20261111T052817Z; RUN=$R/run; B=$R/bag/bag_0.db3; J=/root/spiderx_evidence/s4d_live_journal.md
RR=$(ls $R/restore/restore_verify_*.txt | tail -1); [ -f "$RR" ] || { echo "no restore report"; exit 1; }
grep -q "VERDICT: DISABLED AND VERIFIED" $RR || { echo "restore not verified: no scoring"; exit 1; }
[ -e $R/score ] && { echo "score dir exists: no rerun"; exit 1; }
cd /home/user/spiderx_ws; echo "scorer blob $(git rev-parse HEAD:scripts/s4/score_s4d.py) (HEAD $(git rev-parse HEAD)); worktree entries $(git status --porcelain | wc -l)" > $R/score_provenance.txt
FRESH="env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc -c"
CMD="source /opt/mm/activate.sh >/dev/null 2>&1; source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash; source /home/user/spiderx_ws/install/setup.bash; cd /home/user/spiderx_ws && PYTHONDONTWRITEBYTECODE=1 python3 scripts/s4/score_s4d.py --run $RUN --bag $B --disabled-library $R/disabled/library.json --restore-report $RR --run-install-gates $R/run_install/run_install_gates.json --harness $R/tools/s4d_live_run.sh --out $R/score"
echo "environment: toolchain, r2 overlay, the restored (disabled) workspace installation; command: python3 scripts/s4/score_s4d.py --run $RUN --bag $B --disabled-library $R/disabled/library.json --restore-report $RR --run-install-gates $R/run_install/run_install_gates.json --harness $R/tools/s4d_live_run.sh --out $R/score (no --shadow)" >> $R/score_provenance.txt
t0=$(date +%s); $FRESH "$CMD" > $R/score_stdout.txt 2> $R/score_stderr.txt; SRC=$?
echo "scorer exit $SRC ($(( $(date +%s) - t0 )) s)" | tee -a $R/score_provenance.txt
{ echo "after scoring $(date -u +%FT%TZ):"; sha256sum $B; stat -c '%s bytes' $B; } >> $RUN/bag_hashes.txt
cat $R/score_stdout.txt | cut -c1-600
mkdir -p $R/analysis; $FRESH "source /opt/mm/activate.sh >/dev/null 2>&1; source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash; source /home/user/spiderx_ws/install/setup.bash; cd /home/user/spiderx_ws && PYTHONDONTWRITEBYTECODE=1 python3 $R/tools/analyze_s4d_live_supplementary.py $R" > $R/analysis/run_analysis_supplementary.json 2> $R/analysis/run_analysis_supplementary.stderr; echo "analysis exit $?"
{ echo "after the supplementary analysis $(date -u +%FT%TZ):"; sha256sum $B; stat -c '%s bytes' $B; } >> $RUN/bag_hashes.txt
cat $RUN/bag_hashes.txt
echo "- $(date -u +%FT%TZ) scored (no --shadow): $(tail -1 $R/score_stdout.txt | cut -c1-300); bag hashes: $(grep -c "$(head -2 $RUN/bag_hashes.txt | tail -1 | cut -c1-64)" $RUN/bag_hashes.txt)/3 equal" >> $J
