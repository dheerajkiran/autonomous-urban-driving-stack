#!/usr/bin/env bash
# Start car3d_bridge (WebSocket relay for the browser-based 3D viewer).
# Run this in a second terminal on the VM, alongside vm-run.sh — it's
# optional and only needed if you want the live 3D view.
# Usage (on the VM): ./scripts/vm-run-viewer-bridge.sh

set -e
cd ~/ads_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ads_simulation car3d_bridge
