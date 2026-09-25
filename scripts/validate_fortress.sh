#!/usr/bin/env bash
# Validation of the SpiderX workspace and its Gazebo Fortress simulation.
#
# Run from the workspace root after building and sourcing:
#   colcon build --symlink-install && source install/setup.bash
#   ./scripts/validate_fortress.sh              # static checks (no simulator started)
#   ./scripts/validate_fortress.sh --runtime    # + launch the simulation and check live topics
#   ./scripts/validate_fortress.sh --runtime --headless   # same, Gazebo server without GUI (EGL)
#
# Static checks: package discovery, xacro expansion + check_urdf for every sim_backend,
# URDF->SDF conversion with Fortress's sdformat, world SDF validity, controller config vs URDF,
# Python syntax and argument parsing of every launch file, no Gazebo Classic tokens in the
# Fortress files, and no hardware/serial driver reachable from the simulation launch files.
# Runtime checks: /clock, /scan (frame lidar_link), /joint_states (12 joints), /tf, /tf_static
# are published, and no hardware driver node is running.
set -u
cd "$(dirname "$0")/.."

runtime=0; headless=false
for arg in "$@"; do
  case "$arg" in
    --runtime) runtime=1 ;;
    --headless) headless=true ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg"; exit 2 ;;
  esac
done

fail=0
pass() { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

echo "== Package discovery"
for p in spiderx_description spiderx_bringup spiderx_controller spiderx_localization \
         spiderx_mapping spiderx_navigation spiderx_firmware spiderx_scripts \
         ros_gz_sim ros_gz_bridge ros_gz_interfaces robot_state_publisher xacro; do
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
for f in src/*/launch/*.py; do
  pkg=$(basename "$(dirname "$(dirname "$f")")"); file=$(basename "$f")
  ros2 launch "$pkg" "$file" --show-args > /dev/null 2>&1 \
    && pass "ros2 launch $pkg $file --show-args" || bad "ros2 launch $pkg $file --show-args"
done

echo "== Controller configuration vs URDF"
if out=$(ros2 run spiderx_controller validate_controller_config 2>&1); then
  pass "spiderx_controller configs match the URDF (joints, axes, limits, chains, poses)"
else
  bad "spiderx_controller validation failed:"; echo "$out" | grep FAIL
fi

echo "== No hardware driver in the simulation launch path"
sim_launches=(src/spiderx_bringup/launch/fortress.launch.py src/spiderx_description/launch/fortress.launch.py)
if hits=$(grep -nE 'rplidar|spiderx_firmware|real_robot|/dev/tty|serial_port' "${sim_launches[@]}"); then
  bad "simulation launch files reference hardware:"; echo "$hits"
else
  pass "fortress.launch.py files reference no rplidar / spiderx_firmware / serial device"
fi

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

if [ "$runtime" = 1 ]; then
  echo "== Runtime: ros2 launch spiderx_bringup fortress.launch.py headless:=$headless"
  log="$tmp/launch.log"
  # Own process group, so every child (Gazebo, bridge, ...) can be stopped together.
  setsid ros2 launch spiderx_bringup fortress.launch.py headless:=$headless > "$log" 2>&1 &
  lpid=$!
  stop_sim() { kill -INT -- -"$lpid" 2>/dev/null; sleep 8; kill -KILL -- -"$lpid" 2>/dev/null; }
  if timeout 120 bash -c 'until ros2 topic echo /clock --once >/dev/null 2>&1; do sleep 2; done'; then
    pass "/clock is published"
    sleep 10  # let the robot spawn and the sensors start
    topics=$(ros2 topic list 2>/dev/null)
    for t in /scan /joint_states /tf /tf_static /robot_description; do
      grep -qx "$t" <<< "$topics" && pass "topic $t exists" || bad "topic $t missing"
    done
    # ros2 topic echo may print QoS notices (e.g. 'A message was lost!!!') before the data.
    frame=$(timeout 30 ros2 topic echo /scan --once --field header.frame_id 2>/dev/null \
            | grep -vE 'lost|^---' | head -1)
    [ "$frame" = "lidar_link" ] && pass "/scan frame_id is lidar_link" \
      || bad "/scan frame_id is '$frame' (expected lidar_link)"
    names=$(timeout 30 ros2 topic echo /joint_states --once --field name 2>/dev/null \
            | grep -m1 '^\[')
    n=$(grep -o "'" <<< "$names" | wc -l)
    [ "$n" = 24 ] && pass "/joint_states carries 12 joints" || bad "/joint_states: $names"
    nodes=$(ros2 node list 2>/dev/null)
    if grep -qiE 'rplidar|serial' <<< "$nodes"; then
      bad "hardware driver node running in simulation: $(grep -iE 'rplidar|serial' <<< "$nodes")"
    else
      pass "no hardware driver node running (nodes: $(tr '\n' ' ' <<< "$nodes"))"
    fi
  else
    bad "/clock not published within 120 s; last launch output:"; tail -20 "$log"
  fi
  stop_sim
fi

echo
if [ "$fail" = 0 ]; then echo "All checks passed."; else echo "Some checks FAILED."; fi
exit "$fail"
