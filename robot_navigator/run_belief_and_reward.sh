#!/bin/bash
# Bundled launch: global_feedback + one belief node per robot_id, in a
# single container. Couples their lifecycles — restarting this container
# re-seeds every belief posterior against fresh targets (a belief's
# learned gradient is only valid for the current reward landscape, and
# `forgetting=0.999` is too slow to wash out a stale posterior).
#
# The navigator is intentionally kept separate (./run_navigator.sh) so
# robots can coast during a belief re-seed instead of being killed
# mid-transit.
#
# Usage:
#   ./run_belief_and_reward.sh [--rebuild] [robot_ids_csv] [formation] [spacing] [assignment] [publish_waypoints]
#
# Examples:
#   ./run_belief_and_reward.sh                               # launch-file defaults
#   ./run_belief_and_reward.sh 0,1,2 line 0.4 ordered true
#   ./run_belief_and_reward.sh 0,1,2 triangle 0.5 ordered
#   ./run_belief_and_reward.sh --rebuild 0,1 line 0.3 ordered true

set -e

REBUILD=0
if [ "$1" = "--rebuild" ] || [ "$1" = "-r" ]; then
    REBUILD=1
    shift
fi

ROBOT_IDS="${1:-}"
FORMATION="${2:-}"
SPACING="${3:-}"
ASSIGNMENT="${4:-}"
PUBLISH_WAYPOINTS="${5:true}"

if [ "${REBUILD}" = "1" ]; then
    echo "Rebuilding image from scratch..."
    docker compose build --no-cache
else
    docker compose build
fi
# If build fails on Jetson with an iptables `raw` table error, run:
#   DOCKER_BUILDKIT=0 docker build --network=host -t robot_navigator-robot_navigator .
# or ensure `iptable_raw` is loaded: `sudo modprobe iptable_raw`

# Only forward args the user actually supplied; anything omitted falls
# back to the launch file's defaults.
LAUNCH_ARGS=()
[ -n "${ROBOT_IDS}"         ] && LAUNCH_ARGS+=("robot_ids:=${ROBOT_IDS}")
[ -n "${FORMATION}"         ] && LAUNCH_ARGS+=("formation:=${FORMATION}")
[ -n "${SPACING}"           ] && LAUNCH_ARGS+=("formation_spacing:=${SPACING}")
[ -n "${ASSIGNMENT}"        ] && LAUNCH_ARGS+=("assignment:=${ASSIGNMENT}")
[ -n "${PUBLISH_WAYPOINTS}" ] && LAUNCH_ARGS+=("publish_waypoints:=${PUBLISH_WAYPOINTS}")

CONTAINER_NAME="belief_and_reward"

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator belief_and_reward.launch.py "${LAUNCH_ARGS[@]}"

echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
echo ""
echo "Reward + per-robot beliefs are up. Launch navigators separately:"
echo "  ./run_navigator.sh 0 1 2"
echo ""
echo "Monitor reward:   ros2 topic echo /global_reward"
echo "Monitor waypoint: ros2 topic echo /robot_0/waypoint"
