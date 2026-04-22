#!/bin/bash
# Launch the global_feedback node, which publishes /global_reward and
# /target_markers based on a configured formation.
#
# Usage:
#   ./run_global_feedback.sh [--rebuild] [formation] [spacing] [assignment]
#
# Defaults come from config/global_feedback.yaml. For fine-grained tuning
# (robot_ids, formation_center, formation_yaw, custom targets, etc.) edit
# that yaml and rebuild with --rebuild.
#
# Examples:
#   ./run_global_feedback.sh
#   ./run_global_feedback.sh triangle 0.6
#   ./run_global_feedback.sh line 0.4 ordered
#   ./run_global_feedback.sh --rebuild triangle 0.6 nearest

set -e

REBUILD=0
if [ "$1" = "--rebuild" ] || [ "$1" = "-r" ]; then
    REBUILD=1
    shift
fi

FORMATION="${1:-}"
SPACING="${2:-}"
ASSIGNMENT="${3:-}"

if [ "${REBUILD}" = "1" ]; then
    echo "Rebuilding image from scratch..."
    docker compose build --no-cache
else
    docker compose build
fi
# If build fails on Jetson with an iptables `raw` table error, run:
#   DOCKER_BUILDKIT=0 docker build --network=host -t robot_navigator-robot_navigator .
# or ensure `iptable_raw` is loaded: `sudo modprobe iptable_raw`

# Build the launch-args list only from values the user actually passed,
# so anything omitted falls back to the yaml default.
LAUNCH_ARGS=()
[ -n "${FORMATION}"  ] && LAUNCH_ARGS+=("formation:=${FORMATION}")
[ -n "${SPACING}"    ] && LAUNCH_ARGS+=("formation_spacing:=${SPACING}")
[ -n "${ASSIGNMENT}" ] && LAUNCH_ARGS+=("assignment:=${ASSIGNMENT}")

CONTAINER_NAME="global_feedback"

# Replace any existing container with the same name.
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator global_feedback.launch.py "${LAUNCH_ARGS[@]}"

echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
echo "Monitor reward:   ros2 topic echo /global_reward"
echo "View targets in rviz: MarkerArray on /target_markers (Fixed Frame = field)"
