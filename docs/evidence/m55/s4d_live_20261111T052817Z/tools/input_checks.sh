#!/usr/bin/env bash
# S4d-M LIVE input checks, at the disabled baseline, BEFORE the watchdog and enabling (the S4d shadow input
# checks, with the S4c level-2 live patch checks; read-only; no build, install, launch or deletion).
# Usage: input_checks.sh <scratch dir>
SP=$1; WS=/home/user/spiderx_ws; OV=/root/spiderx_gz_overlay_ws_r2
EXP_HEAD=9cc1f6844de11abc630387d4c6c441a623463094; TESTED=8f99bcc4ab27294265da91a31ce03999423acc7f
EB=/root/spiderx_live_enabled_20261111T052817Z; INST=$WS/install
FU=$WS/docs/evidence/m55/s4d_fresh_disabled_20261110T071506Z; SH=$WS/docs/evidence/m55/s4d_shadow_20261110T072920Z
S3E=$WS/docs/evidence/m55/s3_free_base_20261101T115002Z
cd $WS
F=""
echo "== input checks $(date -u +%FT%TZ)"
echo "branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD) worktree entries $(git status --porcelain | wc -l) local/* branches $(git branch --list 'local/*' | wc -l) remote local/* refs $(git for-each-ref 'refs/remotes/*/local/*' | wc -l)"
[ "$(git rev-parse --abbrev-ref HEAD)" = claude/spiderx-s4d-preparation ] && [ "$(git rev-parse HEAD)" = $EXP_HEAD ] \
  && [ -z "$(git status --porcelain)" ] && [ -z "$(git branch --list 'local/*')" ] && [ -z "$(git for-each-ref 'refs/remotes/*/local/*')" ] \
  && echo "PASS branch, baseline HEAD, clean worktree, no local/* branch or remote ref" || F="$F git"
NE=$(git diff --name-only $TESTED $EXP_HEAD | grep -v '^docs/' || true)
echo "files changed $TESTED..$EXP_HEAD outside docs/: ${NE:-none}"; [ -z "$NE" ] && echo "PASS the executable inputs equal the tested executable 8f99bcc" || F="$F baseline_source"

echo "== toolchain (installed 2026-10-29 by scripts/env/cloud_env_setup.sh 57cc33f8...)"
sha256sum scripts/env/cloud_env_setup.sh
[ "$(sha256sum < scripts/env/cloud_env_setup.sh | cut -c1-64)" = 57cc33f84f9968dff4960bc85b1c1e7baf60025e42ade94061679673996c6193 ] || F="$F toolchain_script"
CXX=$(bash -c 'source /opt/mm/activate.sh >/dev/null 2>&1; x86_64-conda-linux-gnu-g++ --version | head -1')
echo "compiler: $CXX"; grep -qF "compiler $CXX" $OV/OVERLAY_PROVENANCE.txt && echo "PASS compiler = overlay provenance" || F="$F compiler"
bash -c 'source /opt/mm/activate.sh >/dev/null 2>&1; echo "ROS_DISTRO=$ROS_DISTRO prefix=$CONDA_PREFIX"; ign gazebo --versions 2>&1 | head -2'

echo "== r2 overlay provenance"
cmp $OV/OVERLAY_PROVENANCE.txt $S3E/provenance/OVERLAY_PROVENANCE_r2.txt && echo "PASS overlay provenance = the S3 run's committed copy" || F="$F provenance_file"
for p in 0001-plugin-node-parameter-overrides 0002-destructor-no-second-node-removal; do
  h=$(sha256sum < third_party/gz_ros2_control_overlay/patches/$p.patch | cut -c1-64)
  grep -q "^patch $h $p.patch$" $OV/OVERLAY_PROVENANCE.txt && echo "PASS patch $p $h" || { echo "FAIL patch $p $h"; F="$F patch"; }
done
for L in libgz_ros2_control-system.so libgz_hardware_plugins.so; do
  f=$OV/install/gz_ros2_control/lib/$L; h=$(sha256sum < $f | cut -c1-64)
  grep -q "^library $h $f$" $OV/OVERLAY_PROVENANCE.txt && echo "PASS library $L $h ($(stat -c %y $f | cut -c1-19))" || { echo "FAIL library $L $h"; F="$F library"; }
done
S=$OV/src/gz_ros2_control; echo "overlay source HEAD $(git -C $S rev-parse HEAD)"
[ "$(git -C $S rev-parse HEAD)" = bfc5845aa7f0250cbc384773dba5c9847d609b95 ] || F="$F overlay_head"
REF=0
REFD=$(mktemp -d "$SP/overlay_ref.XXXXXX") && git -C $S archive bfc5845aa7f0250cbc384773dba5c9847d609b95 | tar -x -C $REFD \
  && (cd $REFD && for p in 0001-plugin-node-parameter-overrides 0002-destructor-no-second-node-removal; do git apply --whitespace=nowarn $WS/third_party/gz_ros2_control_overlay/patches/$p.patch || exit 1; done) \
  && REF=1
if [ $REF = 1 ]; then
  D=$(diff -r -q --exclude=.git $REFD $S | grep -v "^Only in $REFD" || true)
  if [ -z "$D" ]; then echo "PASS overlay source: every present file = upstream bfc5845 + 0001 + 0002"; else echo "$D"; F="$F overlay_source"; fi
else echo "FAIL overlay reference tree (archive or patch apply)"; F="$F overlay_ref"; fi
echo "overlay reference tree kept, not deleted: ${REFD:-none}"

echo "== the normal workspace installation (restoration target; rebuilt incrementally by restore_cmd.sh)"
python3 - "$INST" <<'PYEOF' || F="$F normal_runtime"
import hashlib, os, subprocess, sys
WS = '/home/user/spiderx_ws'; INST = sys.argv[1]
def sha(b): return hashlib.sha256(b).hexdigest()
bad, src, gen, n, hooks, caches = [], 0, [], 0, [], []
HOOKS = '/opt/mm/root/envs/humble/share/ament_cmake_core/cmake/environment_hooks/'
for p in sorted(d for d in os.listdir(INST) if os.path.isdir(os.path.join(INST, d))):
    for root, _, files in os.walk(os.path.join(INST, p)):
        for f in files:
            path = os.path.join(root, f); n += 1
            if '/__pycache__/' in path and path.endswith('.pyc'): caches.append(path); continue
            if os.path.islink(path) and os.path.realpath(path).startswith(HOOKS): hooks.append(path); continue
            if os.path.islink(path):
                real = os.path.realpath(path)
                if not os.path.isfile(real): bad.append(('dangling', path)); continue
                if not real.startswith(WS + '/src/'):
                    if real.startswith(WS + '/build/'): gen.append(path); continue
                    bad.append(('outside', path, real)); continue
                rel = os.path.relpath(real, WS)
                head = subprocess.run(['git', '-C', WS, 'show', 'HEAD:' + rel], capture_output=True)
                if head.returncode or sha(head.stdout) != sha(open(real, 'rb').read()): bad.append(('differs_from_HEAD', rel)); continue
                src += 1
            else: gen.append(path)
print(f'files {n}: symlinks into src/ equal to the HEAD blob {src}; generated build outputs {len(gen)}; '
      f'toolchain ament env-hook symlinks {len(hooks)}; bytecode caches {len(caches)}; problems {len(bad)}')
for b in bad[:20]: print('  ', b)
sys.exit(1 if bad or src == 0 else 0)
PYEOF
[ ! -e $EB ] && echo "PASS the B1 bases $EB do not exist yet" || { echo "FAIL $EB exists"; F="$F b1_bases"; }

echo "== the recorded disabled validation (batch step 2), reused: no rebuild, no suite rerun"
(cd $FU && sha256sum -c SHA256SUMS --quiet) && echo "PASS manifest ${FU#$WS/}" || F="$F fu_manifest"
grep -q "head $TESTED worktree 0" $FU/fresh/build_and_test_summary.txt && grep -q "^Summary: 1983 tests, 0 errors, 0 failures, 0 skipped$" $FU/fresh/build_and_test_summary.txt \
  && grep -q "^COLLECTION VERDICT: PASS$" $FU/fresh/build_and_test_summary.txt && grep -q "^STEP 2 VERDICT: PASS$" $FU/STEP2_SUMMARY.txt \
  && echo "PASS 1983 collected tests at $TESTED, 0 failures, 0 skipped; collection PASS; step 2 PASS" || F="$F step2"
NB=$(git diff --name-only $TESTED HEAD -- src/ scripts/ third_party/)
[ -z "$NB" ] && echo "PASS src/, scripts/ and third_party/ unchanged since $TESTED: reuse" || { echo "FAIL changed: $NB"; F="$F stale"; }

echo "== gates and phase library (fresh import from the normal installation: env -i, toolchain, overlay, install; from /)"
G=$(env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash -c "source /opt/mm/activate.sh >/dev/null 2>&1; source $OV/install/local_setup.bash; source $INST/setup.bash; cd / && PYTHONDONTWRITEBYTECODE=1 python3 -c \"
import importlib, os
for mod, name in (('m55_contract','M55_LOCOMOTION_DISPATCH_ENABLED'),('m61_live_contract','M61_LIVE_DISPATCH_ENABLED'),('m6_live_contract','LIVE_DISPATCH_ENABLED')):
    m = importlib.import_module('spiderx_controller.'+mod); print(name, getattr(m, name), os.path.realpath(m.__file__))\"; PYTHONDONTWRITEBYTECODE=1 python3 $WS/docs/evidence/m55/s2_fixed_base_20261029T100210Z/tools/library_fingerprint.py 2>&1 | tail -1")
echo "$G"
[ "$(grep -c " False /home/user/spiderx_ws/src/" <<< "$G")" = 3 ] && echo "PASS three gates False" || F="$F gates"
grep -q '"goals": 54, "fingerprints_sha256": "62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732"' <<< "$G" && echo "PASS phase library 54 goals 62f8a100..." || F="$F library_identity"
echo "config: $(sha256sum < src/spiderx_controller/config/m55_locomotion.yaml | cut -c1-64) src/spiderx_controller/config/m55_locomotion.yaml"
[ "$(sha256sum < src/spiderx_controller/config/m55_locomotion.yaml | cut -c1-64)" = ab0dccd43fefe877fac21e1c51d1fdd0ae197ac456b8c42502b0bb7ba5e42a93 ] && echo "PASS config = the step-2 and shadow config" || F="$F config"

echo "== the modules the S4d driver and scorer import (in the toolchain + normal installation), each = its HEAD blob"
MODS=$(env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash -c "source /opt/mm/activate.sh >/dev/null 2>&1; source $OV/install/local_setup.bash; source $INST/setup.bash; cd / && PYTHONDONTWRITEBYTECODE=1 python3 -c \"
import os, sys
sys.path[:0] = ['$WS/scripts/s4', '$WS/scripts/s3', '$WS/scripts/s2']
import s4d_driver, score_s4d, adjudicate_s2, response_law
for k, m in sorted(sys.modules.items()):
    f = getattr(m, '__file__', None) or ''
    if f.startswith('$WS/scripts/') or f.startswith('$WS/docs/'):
        print(os.path.relpath(f, '$WS'))
\"" 2>&1)
echo "imported: $(echo $MODS)"
grep -q "scripts/s4/s4d_driver.py" <<< "$MODS" && grep -q "scripts/s4/score_s4d.py" <<< "$MODS" || F="$F tool_imports"
for p in $MODS; do
  w=$(sha256sum < $p | cut -c1-64); h=$(git show HEAD:$p | sha256sum | cut -c1-64)
  if [ "$w" = "$h" ] && grep -q "^$w  $p = HEAD blob" $SH/provenance/input_checks.txt; then echo "$w  $p = HEAD blob = the S4d shadow run's"
  else echo "$w  $p DIFFERS (HEAD $h or not the shadow run's)"; F="$F tool_$p"; fi
done

echo "== enabling patch (applied only on the local enabling branch, by enable_build.sh)"
P=docs/patches/m55_enable_s2.UNAPPLIED.patch; h=$(sha256sum < $P | cut -c1-64); echo "$h  $P"
[ "$h" = d1dacac5d9b2bc0b63220c3583526d45f5edacb657394b508710978972b20236 ] && echo "PASS patch hash (the S4c live attempts' patch)" || F="$F patch_hash"
git apply --check $P && echo "PASS git apply --check (applies forward to the disabled tree)" || F="$F patch_apply"
NS=$(git apply --numstat $P); echo "$NS"
[ "$(echo "$NS" | wc -l)" = 2 ] && grep -qP "^1\t1\tsrc/spiderx_controller/spiderx_controller/m55_contract.py$" <<< "$NS" \
  && grep -qP "^1\t1\tsrc/spiderx_controller/test/m55_gate.py$" <<< "$NS" && echo "PASS exactly the M55 gate and its test pin" || F="$F patch_scope"

echo "== the S4d shadow evidence (committed d4fe748) vs now"
(cd $SH && sha256sum -c SHA256SUMS --quiet) && echo "PASS shadow evidence manifest" || F="$F shadow_manifest"
for x in 3a8f20441caad3caff4df8912c04e673948f6cf011e34658e5d094c614031b27 06777e140ea1858b1f54e95bd095bf1132b43403a86e9a00dcc2e4de0083d9d5 62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732; do
  grep -q $x $SH/provenance/input_checks.txt && echo "PASS shadow inputs record ${x:0:16}... (same as now)" || { echo "FAIL $x"; F="$F shadow_input"; }; done
grep -q "mapped-object sha256=3a8f2044" $SH/run/premotion/library_checks.txt && grep -q "mapped-object sha256=06777e14" $SH/run/premotion/library_checks.txt \
  && echo "PASS the shadow launch's server mapped these r2 libraries" || F="$F shadow_maps"
a=$(sha256sum < $SH/node/library.json | cut -c1-64); [ "$a" = 917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e ] && echo "PASS shadow node library.json 917fc4d7..." || F="$F library_json"

echo "== OD16 storage before the build (B1 bases measured by step 2: 11,154,692 B) and the recording requirement at T = 7200 s"
python3 - <<'PYEOF' || F="$F storage"
import os
st = os.statvfs('/root/spiderx_evidence'); avail = st.f_bavail * st.f_frsize
b1 = 11154692; E = 2 * 30.8e6 / 60 * 7200; need = max(5 * 2**30, 2 * E + 0.1e9) + b1
print(f'avail {avail} B; need max(5 GiB, 2E + 0.1 GB) + B1 = {need:.0f} B (E = 2 x 30.8 MB/min, a PLANNING ASSUMPTION, x 7200 s)'
      f' -> {"PASS" if avail >= need else "FAIL"}')
raise SystemExit(0 if avail >= need else 1)
PYEOF
echo "== processes"
LEFT=$(pgrep -af "ign gazebo|gz sim|ruby|parameter_bridge|robot_state_publisher|ros2 launch|_ros2_daemon|Xvfb|m55_|s[23]_driver|s4[abcd]_driver|ros2 bag|bag record|strace|watchdog.sh" | grep -v -E "pgrep|bash -c|input_checks|claude/shell-snapshots" || true)
echo "${LEFT:-no leftover processes}"; [ -z "$LEFT" ] || F="$F processes"
[ -e /tmp/.X77-lock ] && { echo "display :77 lock present"; F="$F display"; } || echo "display :77 free"
[ -z "$F" ] && echo "INPUT VERDICT: PASS" || echo "INPUT VERDICT: FAIL:$F"
