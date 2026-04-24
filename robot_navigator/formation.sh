#!/bin/bash
# Drive every visible robot to a preset formation.
# Navigators (./run_navigator.sh <ids>) must already be running.
#
# Each shape places robots at fixed, reproducible positions so the same
# command always produces the same test setup.
#
# Usage:
#   ./formation.sh <corners|line|triangle|circle> [--timeout N] [--tolerance N]

set -e

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <corners|line|triangle|circle> [--timeout N] [--tolerance N]"
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

cd "$(dirname "$0")"

docker compose run --rm robot_navigator \
    ros2 run robot_navigator go_to_starts_lite \
    --shape "$SHAPE" "$@"
