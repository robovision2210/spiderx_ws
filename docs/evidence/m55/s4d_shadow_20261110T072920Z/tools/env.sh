# Sourced by BOTH "terminals" (the launch shell A and the observer/CLI shell B).
# RoboStack ROS 2 Humble (the equivalent of /opt/ros/humble/setup.bash) + the fresh overlay.
source /opt/mm/activate.sh >/dev/null 2>&1
# S3 shadow run-local change (the only one): the verified r2 gz_ros2_control overlay (0.7.15 +
# patches 0001 and 0002), after the toolchain and before the workspace, so ament, pluginlib and
# the launch resolve it first.
source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash
# S4d-M shadow change (the only one): the workspace layer is the fresh DISABLED run installation,
# built and tested at 8f99bcc in new bases (batch step 2, --symlink-install), not the existing
# workspace installation /home/user/spiderx_ws/install, which is not sourced.
source /root/spiderx_fresh_disabled_20261110T071506Z/install/setup.bash
# Discovery: identical in every shell. Domain 0 as documented (observer --domain-id 0);
# localhost-only so nothing outside this machine can join or be joined.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
unset RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER
# Gazebo GUI on the virtual X display of this run (no physical display in this VM).
export DISPLAY=:77
