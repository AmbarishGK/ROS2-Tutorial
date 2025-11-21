#!/usr/bin/env bash
set -e
# Source ROS + workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ws/install/setup.bash
exec "$@"
