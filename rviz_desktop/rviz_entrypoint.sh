#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
rviz2

exec "$@"