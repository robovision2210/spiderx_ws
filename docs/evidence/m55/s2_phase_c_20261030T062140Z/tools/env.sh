# Sourced by BOTH "terminals" (the launch shell A and the observer/CLI shell B).
# RoboStack ROS 2 Humble (the equivalent of /opt/ros/humble/setup.bash) + the fresh overlay.
source /opt/mm/activate.sh >/dev/null 2>&1
# Phase B run-local change (the only one): the verified gz_ros2_control overlay, after the
# toolchain and before the workspace, so ament, pluginlib and the launch resolve it first.
source /root/spiderx_gz_overlay_ws/install/local_setup.bash
source /home/user/spiderx_ws/install/setup.bash
# Discovery: identical in every shell. Domain 0 as documented (observer --domain-id 0);
# localhost-only so nothing outside this machine can join or be joined.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
unset RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER
# Gazebo GUI on the virtual X display of this run (no physical display in this VM).
export DISPLAY=:77
