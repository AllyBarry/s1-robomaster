#!/bin/bash
# Launch one navigator instance per robot_id passed on the command line.
# When more than one ID is given, each instance automatically gets the
# other IDs wired in as peers for reactive collision avoidance. Pass
# --no-collision to skip peer wiring (repulsion layer stays dormant).
#
# Usage:
#   ./run_navigator.sh [--no-collision] <robot_id> [robot_id ...]
#
# Examples:
#   ./run_navigator.sh 0 1 2                    # peers wired across all three
#   ./run_navigator.sh --no-collision 0 1 2     # peers empty, no repulsion

set -e

NO_COLLISION=0
IDS=()

while [ "$#" -gt 0 ]; do
    case "$1" in
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
    echo "Usage: $0 [--no-collision] <robot_id> [robot_id ...]"
    exit 1
fi

docker compose build
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

    LAUNCH_ARGS=("robot_id:=${id}")
    [ -n "${peers}" ] && LAUNCH_ARGS+=("peer_robot_ids:=${peers}")

    docker compose run --remove-orphans -d \
        --name "robot_navigator_${id}" \
        robot_navigator \
        ros2 launch robot_navigator navigator.launch.py "${LAUNCH_ARGS[@]}"
done

echo ""
if [ "${NO_COLLISION}" = "1" ]; then
    echo "Collision avoidance: OFF (no peers passed to any navigator)"
else
    echo "Collision avoidance: ON (each navigator repelled by peers ${IDS[*]})"
fi
