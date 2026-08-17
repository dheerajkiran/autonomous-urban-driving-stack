#!/usr/bin/env bash
# Build the workspace on the VM. Run this after every sync-to-vm.sh.
# Usage (on the VM): ./scripts/vm-build.sh
#
# Not `set -u` — ROS2's setup.bash references variables that don't exist
# yet on first source, so nounset mode fails on a clean shell.
set -e

cd ~/ads_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

echo "Build complete."
