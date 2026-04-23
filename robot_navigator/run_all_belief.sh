#!/bin/bash
# Spin up belief nodes for multiple robots at once. Each robot's belief
# node runs in its own container. By default every robot gets the other
# IDs wired in as peers for repulsive collision avoidance — pass
# --no-collision to disable (useful when isolating learning dynamics
# from the reactive repulsion layer).
#
# Usage:
#   ./run_all_belief.sh [--rebuild] [--with-waypoints] [--no-collision] <id> [id ...]
#
# Examples:
#   ./run_all_belief.sh 0 1 2                              # viz-only, peers wired
#   ./run_all_belief.sh --with-waypoints 0 1 2             # viz + UCB waypoints
#   ./run_all_belief.sh --with-waypoints --no-collision 0 1 2
#   ./run_all_belief.sh --rebuild --with-waypoints 0 1 2   # force image rebuild first

set -e

REBUILD=0
WITH_WAYPOINTS="false"
NO_COLLISION=1
IDS=()

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rebuild|-r)
            REBUILD=1
            shift
            ;;
        --with-waypoints|-w)
            WITH_WAYPOINTS="true"
            shift
            ;;
        --no-collision)
            NO_COLLISION=1
            shift
            ;;
        -*)
            echo "Unknown flag: $1"
            exit 1
            ;;
        *)
            IDS+=("$1")
            shift
            ;;
    esac
done

if [ "${#IDS[@]}" -eq 0 ]; then
    echo "Usage: $0 [--rebuild] [--with-waypoints] [--no-collision] <id> [id ...]"
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

for id in "${IDS[@]}"; do
    peers=""
    if [ "${NO_COLLISION}" = "0" ]; then
        for other in "${IDS[@]}"; do
            if [ "${other}" != "${id}" ]; then
                peers="${peers}${peers:+,}${other}"
            fi
        done
    fi

    CONTAINER_NAME="belief_${id}"
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

    LAUNCH_ARGS=(
        "robot_id:=${id}"
        "publish_waypoints:=${WITH_WAYPOINTS}"
    )
    [ -n "${peers}" ] && LAUNCH_ARGS+=("peer_robot_ids:=${peers}")

    docker compose run --remove-orphans -d \
        --name "${CONTAINER_NAME}" \
        robot_navigator \
        ros2 launch robot_navigator belief.launch.py "${LAUNCH_ARGS[@]}"

    echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
done

echo ""
if [ "${NO_COLLISION}" = "1" ]; then
    echo "Collision avoidance: OFF (no peers passed to any belief node)"
else
    echo "Collision avoidance: ON (each robot repelled by peers ${IDS[*]})"
fi
if [ "${WITH_WAYPOINTS}" = "true" ]; then
    echo "Waypoint publishing: ON — start navigators with:"
    echo "  ./run_navigator.sh ${IDS[*]}"
fi
