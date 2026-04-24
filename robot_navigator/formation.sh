#!/bin/bash
# Drive every visible robot to a preset formation.
# Navigators (./run_navigator.sh <ids>) must already be running.
#
# Usage:
#   ./formation.sh <corners|line|triangle|circle> [--distance N] [--timeout N] [--tolerance N]
#
# --distance is an alias for --spacing (meters). Ignored by 'corners'.
#
# Examples:
#   ./formation.sh corners
#   ./formation.sh triangle --distance 1.0
#   ./formation.sh circle --distance 0.8 --timeout 60

set -e

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <corners|line|triangle|circle> [--distance N] [--timeout N] [--tolerance N]"
    exit 1
fi

SHAPE="$1"
shift

case "$SHAPE" in
    corners|line|triangle|circle) ;;
    *)
        echo "Unknown shape: $SHAPE — use corners, line, triangle, or circle."
        exit 1
        ;;
esac

ARGS=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --distance)
            ARGS+=("--spacing" "$2")
            shift 2
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

cd "$(dirname "$0")"

docker compose run --rm robot_navigator \
    ros2 run robot_navigator go_to_starts_lite \
    --shape "$SHAPE" "${ARGS[@]}"
