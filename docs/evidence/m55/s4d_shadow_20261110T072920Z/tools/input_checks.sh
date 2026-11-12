#!/usr/bin/env bash
# S4d-M shadow input checks (the S4c level-2 shadow checks, adapted; read-only; no build, install,
# launch or deletion). Usage: input_checks.sh <scratch dir>
# Branch/HEAD/worktree; toolchain and r2 overlay provenance (both patches, both built libraries,
# overlay source = upstream + patches, extracted into a NEW unique scratch directory that is kept);
# S4d changes: the installed runtime is the fresh DISABLED run installation built and tested at
# 8f99bcc in batch step 2 (its symlinks vs HEAD, its recorded verdicts); fresh-import gates and the
# phase library from that installation; the modules the S4d driver and scorer actually import, each
# = its HEAD blob, and the unchanged ones = the tools of the S4c level-2 shadow run.
SP=$1; WS=/home/user/spiderx_ws; OV=/root/spiderx_gz_overlay_ws_r2
EXP_HEAD=8f99bcc4ab27294265da91a31ce03999423acc7f
FB=/root/spiderx_fresh_disabled_20261110T071506Z; INST=$FB/install
FU=/root/spiderx_evidence/s4d_fresh_disabled_20261110T071506Z
S3E=$WS/docs/evidence/m55/s3_free_base_20261101T115002Z
L2S=$WS/docs/evidence/m55/s4c_l2_shadow_20261103T094244Z
cd $WS
F=""
echo "== input checks $(date -u +%FT%TZ)"
echo "branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD) worktree entries $(git status --porcelain | wc -l) local/* branches $(git branch --list 'local/*' | wc -l)"
[ "$(git rev-parse --abbrev-ref HEAD)" = claude/spiderx-s4d-preparation ] && [ "$(git rev-parse HEAD)" = $EXP_HEAD ] \
  && [ -z "$(git status --porcelain)" ] && [ -z "$(git branch --list 'local/*')" ] && echo "PASS branch, HEAD, clean worktree, no local/* branch" || F="$F git"

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

echo "== the run installation: the fresh DISABLED build of 8f99bcc in new bases (batch step 2; layout $(stat -c %y $INST/.colcon_install_layout | cut -c1-19), setup $(stat -c %y $INST/local_setup.bash | cut -c1-19))"
python3 - "$INST" <<'PYEOF' || F="$F runtime"
import hashlib, os, subprocess, sys
WS = '/home/user/spiderx_ws'; INST = sys.argv[1]; FB = os.path.dirname(INST)
def sha(b): return hashlib.sha256(b).hexdigest()
bad, src, gen, n = [], 0, [], 0
hooks, caches = [], []    # toolchain ament env hooks (symlinked by colcon); Python bytecode caches
HOOKS = '/opt/mm/root/envs/humble/share/ament_cmake_core/cmake/environment_hooks/'
for p in sorted(d for d in os.listdir(INST) if os.path.isdir(os.path.join(INST, d))):
    for root, _, files in os.walk(os.path.join(INST, p)):
        for f in files:
            path = os.path.join(root, f); n += 1
            if '/__pycache__/' in path and path.endswith('.pyc'):
                caches.append(path); continue          # revalidated against the source by Python
            if os.path.islink(path) and os.path.realpath(path).startswith(HOOKS):
                hooks.append(path); continue
            if os.path.islink(path):
                real = os.path.realpath(path)
                if not os.path.isfile(real):
                    bad.append(('dangling', path)); continue
                if not real.startswith(WS + '/src/'):
                    if real.startswith(FB + '/build/'):
                        gen.append(path); continue
                    bad.append(('outside', path, real)); continue
                rel = os.path.relpath(real, WS)
                head = subprocess.run(['git', '-C', WS, 'show', 'HEAD:' + rel], capture_output=True)
                if head.returncode or sha(head.stdout) != sha(open(real, 'rb').read()):
                    bad.append(('differs_from_HEAD', rel)); continue
                src += 1
            else:
                gen.append(path)
late = [g for g in gen if os.path.getmtime(g) > os.path.getmtime(INST + '/local_setup.bash') + 60]
print(f'files {n}: symlinks into src/ equal to the HEAD blob {src}; generated build outputs {len(gen)} '
      f'(modified after the build: {len(late)}); toolchain ament env-hook symlinks {len(hooks)}; '
      f'bytecode caches {len(caches)}; problems {len(bad)}')
for b in bad[:20]: print('  ', b)
for g in late[:20]: print('  late:', g)
key = ['spiderx_bringup/share/spiderx_bringup/launch/fortress_m55_walking.launch.py',
       'spiderx_controller/lib/python3.11/site-packages/spiderx_controller/m55_locomotion.py',
       'spiderx_controller/lib/python3.11/site-packages/spiderx_controller/m55_rclpy.py',
       'spiderx_controller/lib/python3.11/site-packages/spiderx_controller/m55_teleop.py',
       'spiderx_controller/lib/python3.11/site-packages/spiderx_controller/m55_contract.py',
       'spiderx_controller/lib/spiderx_controller/m55_teleop_keyboard',
       'spiderx_controller/lib/spiderx_controller/m55_locomotion_node']
for k in key:
    p = os.path.join(INST, k)
    print(f'  {k} -> {os.path.relpath(os.path.realpath(p), WS) if os.path.exists(p) else "MISSING"}')
missing = [k for k in key if not os.path.exists(os.path.join(INST, k))]
sys.exit(1 if bad or late or missing or src == 0 else 0)
PYEOF
NB=$(git diff --name-only $EXP_HEAD HEAD -- src/ scripts/ third_party/)
[ -z "$NB" ] && echo "PASS src/, scripts/ and third_party/ = the tested executable SHA $EXP_HEAD (the fresh build's HEAD)" || { echo "FAIL changed since the fresh build: $NB"; F="$F fresh_build_stale"; }
echo "== the recorded step-2 result (no rebuild, no test rerun here)"
grep -h -E "head |Summary:|COLLECTION VERDICT|VERDICT" $FU/fresh/build_and_test_summary.txt $FU/install_A/install_check.txt $FU/install_B/install_check.txt $FU/STEP2_SUMMARY.txt
grep -q "head $EXP_HEAD worktree 0" $FU/fresh/build_and_test_summary.txt && grep -q "^Summary: 1983 tests, 0 errors, 0 failures, 0 skipped$" $FU/fresh/build_and_test_summary.txt \
  && grep -q "^COLLECTION VERDICT: PASS$" $FU/fresh/build_and_test_summary.txt && grep -q "^INSTALLATION A VERDICT: PASS$" $FU/install_A/install_check.txt \
  && grep -q "^INSTALLATION B VERDICT: PASS$" $FU/install_B/install_check.txt && grep -q "^STEP 2 VERDICT: PASS$" $FU/STEP2_SUMMARY.txt \
  && echo "PASS step 2 at $EXP_HEAD: 1983 tests, 0 failures, collection PASS, installations A and B PASS" || F="$F step2"

echo "== gates and phase library (fresh import from the run installation: env -i, toolchain, overlay, then only $INST; from /)"
G=$(env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash -c "source /opt/mm/activate.sh >/dev/null 2>&1; source $OV/install/local_setup.bash; source $INST/setup.bash; cd / && python3 -c \"
import importlib, os
for mod, name in (('m55_contract','M55_LOCOMOTION_DISPATCH_ENABLED'),('m61_live_contract','M61_LIVE_DISPATCH_ENABLED'),('m6_live_contract','LIVE_DISPATCH_ENABLED')):
    m = importlib.import_module('spiderx_controller.'+mod); print(name, getattr(m, name), os.path.realpath(m.__file__), m.__file__)\"; python3 $WS/docs/evidence/m55/s2_fixed_base_20261029T100210Z/tools/library_fingerprint.py 2>&1 | tail -1")
echo "$G"
[ "$(grep -c " False /home/user/spiderx_ws/src/spiderx_controller/spiderx_controller/[a-z0-9_]*\.py $INST/spiderx_controller/" <<< "$G")" = 3 ] && echo "PASS three gates False, imported from the run installation (symlinks into src/)" || F="$F gates"
grep -q '"goals": 54, "fingerprints_sha256": "62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732"' <<< "$G" && echo "PASS phase library 54 goals 62f8a100..." || F="$F library_identity"

echo "== the modules the S4d driver and scorer import (in the run environment), each = its HEAD blob"
MODS=$(bash -c "source $(dirname "$(realpath "$0")")/env.sh; cd / && PYTHONDONTWRITEBYTECODE=1 python3 -c \"
import os, sys
sys.path[:0] = ['$WS/scripts/s4', '$WS/scripts/s3', '$WS/scripts/s2']
import s4d_driver, score_s4d, adjudicate_s2, response_law
for k, m in sorted(sys.modules.items()):
    f = getattr(m, '__file__', None) or ''
    if f.startswith('$WS/scripts/') or f.startswith('$WS/docs/'):
        print(os.path.relpath(f, '$WS'))
\"" 2>&1)
echo "imported: $(echo $MODS)"
[ -n "$MODS" ] && grep -q "scripts/s4/s4d_driver.py" <<< "$MODS" && grep -q "scripts/s4/score_s4d.py" <<< "$MODS" || F="$F tool_imports"
for p in $MODS; do
  w=$(sha256sum < $p | cut -c1-64); h=$(git show HEAD:$p | sha256sum | cut -c1-64)
  if [ "$w" = "$h" ]; then
    if grep -q "^$w  $p = HEAD blob" $L2S/provenance/input_checks.txt; then echo "$w  $p = HEAD blob = the S4c level-2 shadow run's"
    else LC=$(git log --format=%H -1 -- $p)
      if git merge-base --is-ancestor $LC 0cb2cccfd60864809ef44f104ffea2ded7ee1e95; then echo "$w  $p = HEAD blob, unchanged since the S4c level-2 shadow baseline 0cb2ccc (last commit ${LC:0:7}; not in that run's recorded tool list)"
      else echo "$w  $p = HEAD blob, new or changed after the S4c level-2 shadow baseline 0cb2ccc (last commit ${LC:0:7})"; fi; fi
  else echo "$w  $p DIFFERS from HEAD"; F="$F tool_$p"; fi
done
echo "== the restore script (verify-only) and its tests at HEAD"
for p in scripts/m61_restore_and_verify_disabled.sh src/spiderx_controller/test/test_m61_restore_script.py; do
  w=$(sha256sum < $p | cut -c1-64); h=$(git show HEAD:$p | sha256sum | cut -c1-64); echo "$w  $p $([ "$w" = "$h" ] && echo '= HEAD blob' || echo 'DIFFERS')"; [ "$w" = "$h" ] || F="$F $p"
done
echo "== the frozen phase library the bench tests used (read only)"
FL=$WS/docs/evidence/m55/s4c_l1_live_20261102T182303Z/disabled/library.json
[ "$(sha256sum < $FL | cut -c1-64)" = 917fc4d796efe2344d0edcf038a1f7186d053622df250d9b389690a87d485d8e ] && echo "PASS $FL 917fc4d7..." || F="$F frozen_library"
echo "== historical manifests"
for d in $L2S $WS/docs/evidence/m55/s4c_l2_live_*/; do (cd $d && sha256sum -c SHA256SUMS --quiet) && echo "PASS manifest ${d#$WS/}" || { echo "FAIL manifest $d"; F="$F manifest"; }; done
echo "== processes"
LEFT=$(pgrep -af "ign gazebo|gz sim|ruby|parameter_bridge|robot_state_publisher|ros2 launch|_ros2_daemon|Xvfb|m55_|s[23]_driver|s4[abcd]_driver|ros2 bag|bag record|strace|watchdog.sh" | grep -v -E "pgrep|bash -c|input_checks" || true)
echo "${LEFT:-no leftover processes}"; [ -z "$LEFT" ] || F="$F processes"
[ -e /tmp/.X77-lock ] && { echo "display :77 lock present"; F="$F display"; } || echo "display :77 free"
[ -z "$F" ] && echo "INPUT VERDICT: PASS" || echo "INPUT VERDICT: FAIL:$F"
