#!/usr/bin/env bash
# Static validation of the SpiderX Gazebo Fortress setup.
#
# Run from the workspace root after building and sourcing:
#   colcon build --symlink-install && source install/setup.bash
#   ./scripts/validate_fortress.sh
#
# Checks: package discovery, xacro expansion + check_urdf for every sim_backend,
# URDF->SDF conversion with Fortress's sdformat, world SDF validity, Python
# syntax of all launch files, launch-argument parsing, and absence of Gazebo
# Classic tokens in the Fortress-specific files. It does not start Gazebo.
set -u
cd "$(dirname "$0")/.."

fail=0
pass() { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

echo "== Package discovery"
for p in spiderx_description spiderx_bringup ros_gz_sim ros_gz_bridge ros_gz_interfaces \
         robot_state_publisher xacro; do
  if prefix=$(ros2 pkg prefix "$p" 2>/dev/null); then pass "$p -> $prefix"; else bad "$p not found"; fi
done
command -v ign >/dev/null && pass "ign CLI found ($(ign gazebo --versions 2>/dev/null | tail -1))" \
  || bad "ign CLI (Gazebo Fortress) not found"

echo "== xacro expansion and URDF checks"
xacro_file="$(ros2 pkg prefix spiderx_description)/share/spiderx_description/urdf/spiderx.urdf.xacro"
for backend in classic fortress none; do
  if xacro "$xacro_file" sim_backend:=$backend > "$tmp/$backend.urdf" 2> "$tmp/$backend.err"; then
    pass "xacro sim_backend:=$backend"
  else
    bad "xacro sim_backend:=$backend: $(head -1 "$tmp/$backend.err")"; continue
  fi
  if command -v check_urdf >/dev/null; then
    check_urdf "$tmp/$backend.urdf" > /dev/null 2>&1 && pass "check_urdf ($backend)" \
      || bad "check_urdf ($backend)"
  fi
done
if xacro "$xacro_file" sim_backend:=bogus > /dev/null 2>&1; then
  bad "invalid sim_backend value was accepted"
else
  pass "invalid sim_backend value is rejected"
fi
grep -q '<gazebo' "$tmp/none.urdf" && bad "sim_backend:=none still contains <gazebo> tags" \
  || pass "sim_backend:=none has no simulator tags"

echo "== Gazebo Fortress SDF checks"
if ign sdf -p "$tmp/fortress.urdf" > "$tmp/fortress.sdf" 2> "$tmp/sdf.err"; then
  pass "URDF -> SDF conversion (sdformat $(ign sdf --versions 2>/dev/null | tail -1))"
  ign sdf -k "$tmp/fortress.sdf" > /dev/null 2>&1 && pass "converted robot SDF is valid" \
    || bad "converted robot SDF is invalid"
  grep -q "type='gpu_lidar'" "$tmp/fortress.sdf" && pass "gpu_lidar sensor present" \
    || bad "gpu_lidar sensor missing"
  grep -q "<ignition_frame_id>lidar_link</ignition_frame_id>" "$tmp/fortress.sdf" \
    && pass "lidar frame_id is lidar_link" || bad "lidar frame_id override missing"
  grep -q "JointStatePublisher" "$tmp/fortress.sdf" && pass "JointStatePublisher system present" \
    || bad "JointStatePublisher system missing"
else
  bad "URDF -> SDF conversion failed: $(head -1 "$tmp/sdf.err")"
fi
world="$(ros2 pkg prefix spiderx_description)/share/spiderx_description/worlds/spiderx_fortress.sdf"
ign sdf -k "$world" > /dev/null 2>&1 && pass "world SDF is valid" || bad "world SDF is invalid"

echo "== Mesh URIs (fortress description)"
missing=0
for uri in $(grep -o 'file://[^"]*' "$tmp/fortress.urdf" | sort -u); do
  [ -f "${uri#file://}" ] || { bad "missing mesh ${uri#file://}"; missing=1; }
done
[ "$missing" = 0 ] && pass "all $(grep -o 'file://[^"]*' "$tmp/fortress.urdf" | sort -u | wc -l) mesh files resolve"

echo "== Launch files"
for f in src/*/launch/*.py; do
  python3 -m py_compile "$f" 2>/dev/null && pass "py_compile $f" || bad "py_compile $f"
done
for lf in "spiderx_description fortress.launch.py" "spiderx_bringup fortress.launch.py"; do
  # shellcheck disable=SC2086
  ros2 launch $lf --show-args > /dev/null 2>&1 && pass "ros2 launch $lf --show-args" \
    || bad "ros2 launch $lf --show-args"
done

echo "== No Gazebo Classic tokens in Fortress-specific files"
fortress_files=(
  src/spiderx_bringup/launch/fortress.launch.py
  src/spiderx_description/launch/fortress.launch.py
  src/spiderx_description/worlds/spiderx_fortress.sdf
  src/spiderx_description/config/fortress_bridge.yaml
  src/spiderx_description/urdf/spiderx_fortress.gazebo.xacro
  "$tmp/fortress.urdf"
)
pattern='gazebo_ros|spawn_entity|gzserver|gzclient|GAZEBO_MODEL_PATH|libgazebo_ros_ray_sensor|gazebo_ros2_control'
if hits=$(grep -nE "$pattern" "${fortress_files[@]}"); then
  bad "Classic tokens found:"; echo "$hits"
else
  pass "none of: ${pattern//|/, }"
fi

echo
if [ "$fail" = 0 ]; then echo "All checks passed."; else echo "Some checks FAILED."; fi
exit "$fail"
