#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"
colcon build --symlink-install
source install/setup.bash

echo "Workspace tekbot_ws pret."
