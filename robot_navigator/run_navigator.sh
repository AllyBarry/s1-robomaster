#!/bin/bash
# Launch one navigator instance per robot_id passed on the command line.
# Usage: ./run_navigator.sh 0 1 2

set -e

if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <robot_id> [robot_id ...]"
    exit 1
fi

docker compose build

for id in "$@"; do
    docker compose run --remove-orphans -d \
        --name "robot_navigator_${id}" \
        robot_navigator \
        ros2 launch robot_navigator navigator.launch.py robot_id:=${id}
done
