#!/usr/bin/env bash
# Recreate the Cloud toolchain of the M6.1 / M5.5 runs (2026-10-09..13): RoboStack ROS 2 Humble
# under /opt/mm, Python 3.11, Gazebo Fortress. Pins as recorded in docs/M1_TEST_RESULTS.md,
# docs/M4_1_TEST_RESULTS.md and docs/M55_KEYBOARD_WALKING.md §13. Idempotent; untested as a whole
# (the repo records versions, not the original install command). For the environment's setup
# script: bash spiderx_ws/scripts/env/cloud_env_setup.sh
set -euo pipefail
MM=/opt/mm; ENV=humble; MMV=1.5.10-0
PINS=(python=3.11 "libignition-gazebo6=6.16.0" "ros-humble-ros-gz=0.244.20"
  "ros-humble-ros2-controllers=2.48.0" "ros-humble-joint-trajectory-controller=2.48.0"
  "ros-humble-joint-state-broadcaster=2.48.0" "ros-humble-controller-manager=2.51.0"
  "ros-humble-ros2controlcli=2.51.0" "ros-humble-gz-ros2-control=0.7.15"
  "ros-humble-rclpy=3.3.16" "ros-humble-control-msgs=4.8.0" "numpy=1.26.4")
PKGS=(ros-humble-ros-base ros-humble-ros2-control ros-humble-xacro ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher ros-humble-tf2-ros ros-humble-std-srvs ros-humble-rosbag2
  ros-humble-rosbag2-storage-default-plugins ros-humble-rmw-fastrtps-cpp
  ros-humble-launch-testing-ament-cmake ros-humble-ament-cmake-pytest ros-humble-ament-lint-auto
  ros-humble-ament-lint-common colcon-common-extensions compilers cmake make pkg-config
  pytest pyyaml matplotlib flake8)
mkdir -p "$MM/bin"
if [ ! -x "$MM/bin/micromamba" ]; then       # from conda-forge, not micro.mamba.pm
  curl -fsSL "https://conda.anaconda.org/conda-forge/linux-64/micromamba-$MMV.tar.bz2" \
    | tar -xj -C "$MM" bin/micromamba
fi
export MAMBA_ROOT_PREFIX="$MM/root"
OP=create; [ -d "$MAMBA_ROOT_PREFIX/envs/$ENV" ] && OP=install     # re-run: install is a no-op
"$MM/bin/micromamba" "$OP" -y -n "$ENV" -c robostack-staging -c conda-forge \
  --channel-priority strict "${PINS[@]}" "${PKGS[@]}"
cat > "$MM/activate.sh" <<EOF
export MAMBA_ROOT_PREFIX=$MM/root
eval "\$($MM/bin/micromamba shell hook -s bash)"
micromamba activate $ENV
EOF
# setuptools 59.6.0 for colcon --symlink-install only (PYTHONPATH=$MM/st59site at build time)
[ -d "$MM/st59site/setuptools" ] || "$MM/root/envs/$ENV/bin/python" -m pip install -q \
  --target "$MM/st59site" setuptools==59.6.0
command -v Xvfb >/dev/null || { apt-get update -qq && apt-get install -y -qq xvfb; }
"$MM/bin/micromamba" list -n "$ENV" | tee "$MM/versions.txt" | grep -E \
  " (python|libignition-gazebo6|ros-humble-(ros-gz|joint-trajectory-controller|controller-manager|gz-ros2-control|rclpy)) "
