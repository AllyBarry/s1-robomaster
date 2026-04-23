#!/bin/bash
# Spin up a belief node for a single robot. By default the node only
# visualises its belief — add --with-waypoints to let it also publish
# UCB-driven targets on /robot_{id}/waypoint (the navigator must be
# running for that to actually drive the robot).
#
# Usage:
#   ./run_belief.sh [--rebuild] <robot_id> [--with-waypoints] [--peers 1,2] [--no-collision]
#
# Examples:
#   ./run_belief.sh 0                            # viz-only belief for robot 0
#   ./run_belief.sh 0 --with-waypoints           # viz + UCB waypoints
#   ./run_belief.sh 0 --with-waypoints --peers 1,2   # add repulsion from 1 and 2
#   ./run_belief.sh 0 --with-waypoints --no-collision   # force peers empty (disables repulsion)
#   ./run_belief.sh --rebuild 0                  # force image rebuild first

set -e

REBUILD=0
if [ "$1" = "--rebuild" ] || [ "$1" = "-r" ]; then
    REBUILD=1
    shift
fi

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 [--rebuild] <robot_id> [--with-waypoints] [--peers <csv>]"
    exit 1
fi

ROBOT_ID="$1"
shift

WITH_WAYPOINTS="false"
PEERS=""
NO_COLLISION=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --with-waypoints|-w)
            WITH_WAYPOINTS="true"
            shift
            ;;
        --peers|-p)
            PEERS="$2"
            shift 2
            ;;
        --no-collision)
            NO_COLLISION=1
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

if [ "${NO_COLLISION}" = "1" ]; then
    if [ -n "${PEERS}" ]; then
        echo "--no-collision overrides --peers ${PEERS}; peers will be empty."
    fi
    PEERS=""
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

CONTAINER_NAME="belief_${ROBOT_ID}"
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker compose run --remove-orphans -d \
    --name "${CONTAINER_NAME}" \
    robot_navigator \
    ros2 launch robot_navigator belief.launch.py \
        robot_id:=${ROBOT_ID} \
        publish_waypoints:=${WITH_WAYPOINTS} \
        peer_robot_ids:=${PEERS}

echo "Started ${CONTAINER_NAME} — follow with: docker logs -f ${CONTAINER_NAME}"
echo ""
echo "rviz displays (Fixed Frame = field):"
echo "  Map        on /robot_${ROBOT_ID}/belief/uncertainty"
echo "  Map        on /robot_${ROBOT_ID}/belief/gradient_mag"
echo "  Map        on /robot_${ROBOT_ID}/belief/repulsion"
echo "  MarkerArray on /robot_${ROBOT_ID}/belief/gradients"
if [ "${WITH_WAYPOINTS}" = "true" ]; then
    echo ""
    echo "Waypoint publishing enabled — start navigator for robot ${ROBOT_ID}:"
    echo "  ./run_navigator.sh ${ROBOT_ID}"
fi
if [ "${NO_COLLISION}" = "1" ]; then
    echo "Collision avoidance: OFF (peer repulsion disabled)"
elif [ -n "${PEERS}" ]; then
    echo "Peer repulsion active against robot(s): ${PEERS}"
fi
