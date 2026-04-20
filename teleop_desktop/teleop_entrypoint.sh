#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Friendly hint when running the default command
if [ "$1" = "rqt_robot_steering" ]; then
    echo ""
    echo "Starting rqt_robot_steering..."
    echo "In the GUI, set the Topic field to: /robot_<ID>/cmd_vel"
    echo "(e.g. /robot_4/cmd_vel), then use the sliders to drive."
    echo ""
    exec ros2 run rqt_robot_steering rqt_robot_steering
fi

exec "$@"
