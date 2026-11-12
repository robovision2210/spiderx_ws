#!/usr/bin/env bash
# S4b shadow input checks (the S4a shadow checks, adapted; read-only; no build, install or launch). Usage: input_checks.sh <scratch dir>
# Branch/HEAD/worktree; toolchain and r2 overlay provenance (both patches, both built libraries,
# overlay source = upstream + patches); installed SpiderX runtime vs HEAD; fresh-import gates;
# phase-library identity; S4a driver/scorer identity at HEAD.
SP=$1; WS=/home/user/spiderx_ws; OV=/root/spiderx_gz_overlay_ws_r2
EXP_HEAD=e406590a98ead5052b610316a5d23aba0c6edaf6
S3E=$WS/docs/evidence/m55/s3_free_base_20261101T115002Z
cd $WS
F=""
echo "== input checks $(date -u +%FT%TZ)"
echo "branch $(git rev-parse --abbrev-ref HEAD) HEAD $(git rev-parse HEAD) worktree entries $(git status --porcelain | wc -l) local/* branches $(git branch --list 'local/*' | wc -l)"
[ "$(git rev-parse --abbrev-ref HEAD)" = claude/spiderx-s4b-preparation ] && [ "$(git rev-parse HEAD)" = $EXP_HEAD ] \
  && [ -z "$(git status --porcelain)" ] && [ -z "$(git branch --list 'local/*')" ] || F="$F git"

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
rm -rf $SP/ref && mkdir -p $SP/ref && git -C $S archive bfc5845aa7f0250cbc384773dba5c9847d609b95 | tar -x -C $SP/ref \
  && (cd $SP/ref && for p in 0001-plugin-node-parameter-overrides 0002-destructor-no-second-node-removal; do git apply --whitespace=nowarn $WS/third_party/gz_ros2_control_overlay/patches/$p.patch || exit 1; done) \
  && REF=1
if [ $REF = 1 ]; then
  D=$(diff -r -q --exclude=.git $SP/ref $S | grep -v "^Only in $SP/ref" || true)
  if [ -z "$D" ]; then echo "PASS overlay source: every present file = upstream bfc5845 + 0001 + 0002"; else echo "$D"; F="$F overlay_source"; fi
else echo "FAIL overlay reference tree (archive or patch apply)"; F="$F overlay_ref"; fi
rm -rf $SP/ref

echo "== installed SpiderX runtime vs HEAD (install built $(stat -c %y install/.colcon_install_layout | cut -c1-19) by the S4a live restore's clean rebuild of e5899f4)"
echo "src changes e5899f4..HEAD: $(git diff --name-only e5899f40482891b9eba026c40de97daa3931d84a HEAD -- src/ | tr '\n' ' ')"
python3 - <<'EOF' || F="$F runtime"
import hashlib, os, subprocess, sys
WS = '/home/user/spiderx_ws'; INST = WS + '/install'
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
                    if real.startswith(WS + '/build/'):
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
       'spiderx_controller/lib/python3.11/site-packages/spiderx_controller/m55_contract.py']
for k in key:
    p = os.path.join(INST, k)
    print(f'  {k} -> {os.path.relpath(os.path.realpath(p), WS) if os.path.exists(p) else "MISSING"}')
missing = [k for k in key if not os.path.exists(os.path.join(INST, k))]
sys.exit(1 if bad or late or missing or src == 0 else 0)
EOF
find install -name test_m61_restore_script.py | grep -q . && { echo "FAIL the changed test file is installed"; F="$F installed_test"; } || echo "PASS the only src change since the build (a test file) is not installed"

echo "== gates (fresh import, env -i, from /) and phase library"
G=$(env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash -c 'source /opt/mm/activate.sh >/dev/null 2>&1; source /home/user/spiderx_ws/install/setup.bash; cd / && python3 -c "
import importlib, os
for mod, name in ((\"m55_contract\",\"M55_LOCOMOTION_DISPATCH_ENABLED\"),(\"m61_live_contract\",\"M61_LIVE_DISPATCH_ENABLED\"),(\"m6_live_contract\",\"LIVE_DISPATCH_ENABLED\")):
    m = importlib.import_module(\"spiderx_controller.\"+mod); print(name, getattr(m, name), os.path.realpath(m.__file__))"; python3 /home/user/spiderx_ws/docs/evidence/m55/s2_fixed_base_20261029T100210Z/tools/library_fingerprint.py 2>&1 | tail -1')
echo "$G"
[ "$(grep -c " False /home/user/spiderx_ws/src/" <<< "$G")" = 3 ] || F="$F gates"
grep -q '"goals": 54, "fingerprints_sha256": "62f8a10075e90571bfc42afb4635eddd235d8681f589f027734fa5ca2c03e732"' <<< "$G" && echo "PASS phase library 54 goals 62f8a100..." || F="$F library_identity"

echo "== S4b tools at HEAD (s4b_driver imports s4a_driver, s3_driver, s2_driver; score_s4b imports score_s4a, score_s3)"
for p in scripts/s4/s4b_driver.py scripts/s4/score_s4b.py scripts/s4/s4a_driver.py scripts/s4/score_s4a.py scripts/s3/s3_driver.py scripts/s2/s2_driver.py scripts/s3/score_s3.py; do
  w=$(sha256sum < $p | cut -c1-64); h=$(git show HEAD:$p | sha256sum | cut -c1-64)
  echo "$w  $p $([ "$w" = "$h" ] && echo '= HEAD blob' || { echo 'DIFFERS from HEAD'; })"; [ "$w" = "$h" ] || F="$F tool_$p"
done
grep -q "^0af5b90a410acfcb4b7e3967d1aaff0d8eb2e23c441aea83fe7bbf4233c612e7  scripts/s3/s3_driver.py" $S3E/provenance/input_checks.txt \
  && [ "$(sha256sum < scripts/s3/s3_driver.py | cut -c1-64)" = 0af5b90a410acfcb4b7e3967d1aaff0d8eb2e23c441aea83fe7bbf4233c612e7 ] \
  && [ "$(sha256sum < scripts/s2/s2_driver.py | cut -c1-64)" = 44cd495913b413c3f984adb581c699258a6da5cc7495c820ce3ef5e92ed78388 ] \
  && echo "PASS s3_driver.py and s2_driver.py = the drivers that ran S3 (P4 and live)" || F="$F base_drivers"

echo "== processes"
LEFT=$(pgrep -af "ign gazebo|gz sim|ruby|parameter_bridge|robot_state_publisher|ros2 launch|_ros2_daemon|Xvfb|m55_|s[23]_driver|s4a_driver|ros2 bag|strace|watchdog.sh" | grep -v -E "pgrep|bash -c|input_checks" || true)
echo "${LEFT:-no leftover processes}"; [ -z "$LEFT" ] || F="$F processes"
[ -e /tmp/.X77-lock ] && { echo "display :77 lock present"; F="$F display"; } || echo "display :77 free"
[ "$(sha256sum < scripts/s4/s4a_driver.py | cut -c1-64)" = 243974e75a5784b3a482364f29e6e1b978cc24e12a847927f2c2bfb00643bc50 ] \
  && [ "$(sha256sum < scripts/s4/score_s4a.py | cut -c1-64)" = 631fe2ec8cab638780511d919df51617410868508b9073ff61e83aab39d6e66a ] \
  && echo "PASS s4a_driver.py and score_s4a.py = the S4a tools that ran and scored S4a" || F="$F s4a_tools"
I=$(git diff --name-only e5899f40482891b9eba026c40de97daa3931d84a HEAD -- src/ | grep -v '^src/spiderx_controller/test/' || true)
[ -z "$I" ] && echo "PASS since the reused build (e5899f4): no installed src/ file changed (tests only)" || { echo "FAIL src changed since the build: $I"; F="$F build_stale"; }
[ -z "$F" ] && echo "INPUT VERDICT: PASS" || echo "INPUT VERDICT: FAIL:$F"
