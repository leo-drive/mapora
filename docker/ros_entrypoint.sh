#!/bin/bash
set -e
set -x

# setup ros2 and mapora environment
source "/opt/ros/humble/setup.bash" --
source "/root/mapora_ws/install/setup.bash" --
exec "$@"