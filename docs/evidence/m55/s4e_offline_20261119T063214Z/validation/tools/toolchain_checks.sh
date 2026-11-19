#!/usr/bin/env bash
# Post-merge: the documented cloud toolchain and the r2 gz_ros2_control overlay, against the FRESH CLONE's committed
# provenance (read-only; the S4d-E live input checks' toolchain/overlay section, with the clone as the workspace).
# Usage: toolchain_checks.sh <clone> <scratch dir for the overlay reference tree>
WS=$1; SP=$2; OV=/root/spiderx_gz_overlay_ws_r2; S3E=$WS/docs/evidence/m55/s3_free_base_20261101T115002Z; F=""; cd $WS
echo "== toolchain $(date -u +%FT%TZ)"
h=$(sha256sum < scripts/env/cloud_env_setup.sh | cut -c1-64); echo "$h  scripts/env/cloud_env_setup.sh"
[ "$h" = 57cc33f84f9968dff4960bc85b1c1e7baf60025e42ade94061679673996c6193 ] && echo "PASS toolchain setup script = the recorded 57cc33f8..." || F="$F toolchain_script"
CXX=$(bash -c 'source /opt/mm/activate.sh >/dev/null 2>&1; x86_64-conda-linux-gnu-g++ --version | head -1')
echo "compiler: $CXX"; grep -qF "compiler $CXX" $OV/OVERLAY_PROVENANCE.txt && echo "PASS compiler = overlay provenance" || F="$F compiler"
bash -c 'source /opt/mm/activate.sh >/dev/null 2>&1; echo "ROS_DISTRO=$ROS_DISTRO prefix=$CONDA_PREFIX python=$(python3 --version 2>&1)"; ign gazebo --versions 2>&1 | head -1; colcon version-check 2>/dev/null | head -1'
echo "== r2 overlay provenance"
cmp $OV/OVERLAY_PROVENANCE.txt $S3E/provenance/OVERLAY_PROVENANCE_r2.txt && echo "PASS overlay provenance = the clone's committed S3 copy" || F="$F provenance_file"
for p in 0001-plugin-node-parameter-overrides 0002-destructor-no-second-node-removal; do
  h=$(sha256sum < third_party/gz_ros2_control_overlay/patches/$p.patch | cut -c1-64)
  grep -q "^patch $h $p.patch$" $OV/OVERLAY_PROVENANCE.txt && echo "PASS patch $p $h (clone file = provenance)" || { echo "FAIL patch $p $h"; F="$F patch"; }; done
for L in libgz_ros2_control-system.so libgz_hardware_plugins.so; do f=$OV/install/gz_ros2_control/lib/$L; h=$(sha256sum < $f | cut -c1-64)
  grep -q "^library $h $f$" $OV/OVERLAY_PROVENANCE.txt && echo "PASS library $L $h" || { echo "FAIL library $L $h"; F="$F library"; }; done
S=$OV/src/gz_ros2_control; [ "$(git -C $S rev-parse HEAD)" = bfc5845aa7f0250cbc384773dba5c9847d609b95 ] && echo "PASS overlay source HEAD bfc5845" || F="$F overlay_head"
REFD=$(mktemp -d "$SP/overlay_ref.XXXXXX") && git -C $S archive bfc5845aa7f0250cbc384773dba5c9847d609b95 | tar -x -C $REFD \
  && (cd $REFD && for p in 0001-plugin-node-parameter-overrides 0002-destructor-no-second-node-removal; do git apply --whitespace=nowarn $WS/third_party/gz_ros2_control_overlay/patches/$p.patch || exit 1; done) \
  && { D=$(diff -r -q --exclude=.git $REFD $S | grep -v "^Only in $REFD" || true); [ -z "$D" ] && echo "PASS overlay source: every present file = upstream bfc5845 + the clone's 0001 + 0002" || { echo "$D"; F="$F overlay_source"; }; } \
  || { echo "FAIL overlay reference tree"; F="$F overlay_ref"; }
echo "overlay reference tree kept (not deleted): $REFD"
h=$(sha256sum < src/spiderx_controller/config/m55_locomotion.yaml | cut -c1-64); echo "config $h"; [ "$h" = ab0dccd43fefe877fac21e1c51d1fdd0ae197ac456b8c42502b0bb7ba5e42a93 ] && echo "PASS config = the validated ab0dccd4..." || F="$F config"
[ -z "$F" ] && echo "TOOLCHAIN VERDICT: PASS" || echo "TOOLCHAIN VERDICT: FAIL:$F"
