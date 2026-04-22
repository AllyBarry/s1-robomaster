#!/bin/bash
# Launch a follower: robot <follower_id> tracks robot <target_id>, staying
# <follow_distance> metres away.
#
# Usage:
#   ./run_follower.sh [--rebuild] <follower_id> <target_id> [follow_distance]
#
# Examples:
#   ./run_follower.sh 2 1                 # robot 2 follows robot 1 at 0.5m (default)
#   ./run_follower.sh 2 1 0.4             # robot 2 follows robot 1 at 0.4m
#   ./run_follower.sh --rebuild 2 1 0.4   # force a clean image build first

set -e

REBUILD=0
if [ "$1" = "--rebuild" ] || [ "$1" = "-r" ]; then
    REBUILD=1
    shift
fi

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 [--rebuild] <follower_id> <target_id> [follow_distance]"
    exit 1
fi

FOLLOWER_ID="$1"
TARGET_ID="$2"
DISTANCE="${3:-0.5}"

if [ "${FOLLOWER_ID}" = "${TARGET_ID}" ]; then
    echo "Error: follower_id and target_id must differ (a robot can't follow itself)."
    exit 1
fi

if [ "${REBUILD}" = "1" ]; then
    echo "Rebuilding image from scratch..."
    docker compose build --no-cache
else
    docker compose build
fi
# If build fails on Jetson with an iptables `raw` table error, run:
#   DOCKER_BUILDKIT=0 docker build --network=host -t robot_navigator-robot_navigator .
# or ensure `iptable_raw` is loaded: `sudo modprobe iptable_raw`

CONTAINER_NAME="follower_${FOLLOWER_ID}_to_${TARGET_ID}"

# Replace any existing container with the same name so re-runs are clean.
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator follower.launch.py \
        robot_id:=${FOLLOWER_ID} \
        target_robot_id:=${TARGET_ID} \
        follow_distance:=${DISTANCE}

echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
