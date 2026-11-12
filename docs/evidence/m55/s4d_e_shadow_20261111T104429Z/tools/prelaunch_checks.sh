#!/usr/bin/env bash
# S4d-M shadow pre-launch checks (the S4c level-2 shadow checks; the S4c level-1 shadow checks; the S4b shadow checks; the S3 shadow-2 checks; the r2 overlay, and all three gates expected
# False): the run-local env.sh resolves gz_ros2_control to the verified r2 overlay.
# S4d changes: the workspace layer is the fresh disabled run installation; the existing workspace
# installation must not be in the environment, and the gates must be imported from the run
# installation (its symlinks resolve into the workspace src/).
# Usage: prelaunch_checks.sh <tools dir>   Ends with "PRELAUNCH VERDICT: PASS" or "... FAIL".
EVT=$1
source $EVT/env.sh
O=/root/spiderx_gz_overlay_ws_r2/install/gz_ros2_control
F=0
echo "== source order (AMENT_PREFIX_PATH, toolchain/overlay/workspace positions)"
echo "$AMENT_PREFIX_PATH" | tr ":" "\n" | grep -n -E "gz_overlay|envs/humble$|spiderx_fresh_disabled_20261111T104338Z/install/spiderx_(controller|description|bringup)$|spiderx_ws/install"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>} ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-<unset>} DISPLAY=$DISPLAY"
echo "== the existing workspace installation must not be in the environment"
if env | grep -E "^(AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|COLCON_PREFIX_PATH|PYTHONPATH|LD_LIBRARY_PATH|PATH)=" | grep -q "/home/user/spiderx_ws/install"; then echo "FAIL /home/user/spiderx_ws/install is in the environment"; F=1
else echo "PASS /home/user/spiderx_ws/install is not in the environment"; fi
echo "== ament lookup"; p=$(ros2 pkg prefix gz_ros2_control); echo "$p"
[ "$p" = "$O" ] && echo "PASS ament prefix = overlay" || { echo "FAIL ament prefix"; F=1; }
echo "== pluginlib"
python3 -c "from ament_index_python import get_resources as g; r=g(\"gz_ros2_control__pluginlib__plugin\"); print(r); assert r[\"gz_ros2_control\"] == \"$O\"; print(\"PASS pluginlib = overlay\")" || F=1
echo "== plugin XML library path"; grep -o "library path=\"[^\"]*\"" $O/share/gz_ros2_control/gz_hardware_plugins.xml
echo "== dependency resolution"
for L in $O/lib/libgz_ros2_control-system.so $O/lib/libgz_hardware_plugins.so; do nf=$(ldd $L | grep -c "not found"); echo "$(basename $L): unresolved=$nf"; [ "$nf" = 0 ] || F=1; done
DL=$(python3 -c "import ctypes,os; [ctypes.CDLL(os.path.join(\"$O/lib\",n), mode=os.RTLD_NOW|os.RTLD_LOCAL) for n in (\"libgz_hardware_plugins.so\",\"libgz_ros2_control-system.so\")]; print(\"PASS dlopen RTLD_NOW both\")" 2>&1 | grep -v -E "class_loader|SEVERE|at line")
echo "$DL"; grep -q "PASS dlopen" <<< "$DL" || F=1
echo "== plugin search path overrides (must be empty before launch)"
echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-<unset>} GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH:-<unset>}"
[ -z "$IGN_GAZEBO_SYSTEM_PLUGIN_PATH$GZ_SIM_SYSTEM_PLUGIN_PATH" ] || F=1
ls ~/.ignition/gazebo/plugins 2>/dev/null | grep -i ros2_control && F=1 || echo "no gz_ros2_control in ~/.ignition/gazebo/plugins"
ls /opt/mm/root/envs/humble/lib/ign-gazebo-6/plugins 2>/dev/null | grep -i ros2_control && F=1 || echo "no gz_ros2_control in the ign-gazebo default plugin dir"
echo "LD_LIBRARY_PATH entries:"; echo "$LD_LIBRARY_PATH" | tr ":" "\n"
LP=$(python3 -c "import os; from ament_index_python.packages import get_package_prefix as g; print(os.path.join(g(\"gz_ros2_control\"),\"lib\"))")
echo "launch-time plugin dir: $LP"; [ "$LP" = "$O/lib" ] || F=1
echo "== gates (fresh import; expected all three False)"
( cd / && python3 -c "
import importlib, os
want = {\"M55_LOCOMOTION_DISPATCH_ENABLED\": False, \"M61_LIVE_DISPATCH_ENABLED\": False, \"LIVE_DISPATCH_ENABLED\": False}
for mod, name in ((\"m55_contract\",\"M55_LOCOMOTION_DISPATCH_ENABLED\"),(\"m61_live_contract\",\"M61_LIVE_DISPATCH_ENABLED\"),(\"m6_live_contract\",\"LIVE_DISPATCH_ENABLED\")):
    m = importlib.import_module(\"spiderx_controller.\"+mod); v = getattr(m, name)
    print(name, v, m.__file__, \"->\", os.path.realpath(m.__file__)); assert v is want[name] and m.__file__.startswith(\"/root/spiderx_fresh_disabled_20261111T104338Z/install/\") and os.path.realpath(m.__file__).startswith(\"/home/user/spiderx_ws/src/\")
print(\"PASS gates M55 False, M61 False, M6.0-D False, imported from the run installation (symlinks into the workspace src/)\")" ) || F=1
[ "$F" = 0 ] && echo "PRELAUNCH VERDICT: PASS" || echo "PRELAUNCH VERDICT: FAIL"
