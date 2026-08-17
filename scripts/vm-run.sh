#!/usr/bin/env bash
# Launch the main stack on the VM (map loader, route planner, SUMO bridge,
# Pygame click-to-drive viewer). Run in its own terminal — this is the
# foreground process you watch/interact with.
#
# Skips the build by default: with --symlink-install, edits to existing
# .py files take effect immediately without rebuilding. Only pass --build
# after changing a .msg file, package.xml/setup.py, or adding a new
# package/node — those actually need colcon to re-run.
# Usage (on the VM): ./scripts/vm-run.sh [--build]

set -e
cd ~/ads_ws

if [[ "${1:-}" == "--build" ]]; then
  ./scripts/vm-build.sh
else
  source /opt/ros/humble/setup.bash
  source install/setup.bash
fi

ros2 launch launch/ads_full.launch.py
