#!/bin/bash
# Launch the teleop GUI over X11 forwarding.
# Assumes you've SSH'd in with `ssh -X` or `ssh -Y` to the machine running this.
#
# Usage:
#   ./run_teleop.sh             # x86/standard host
#   ./run_teleop.sh --nvidia    # NVIDIA Jetson host (uses L4T base + nvidia runtime)

set -e

SERVICE="teleop"
export BASE_IMAGE="ros:humble-ros-core"
export TELEOP_TAG="x86"

for arg in "$@"; do
    case "$arg" in
        --nvidia|--jetson)
            SERVICE="teleop-jetson"
            export BASE_IMAGE="dustynv/ros:humble-pytorch-l4t-r35.3.1"
            export TELEOP_TAG="jetson"
            echo "Running in NVIDIA Jetson mode (base: $BASE_IMAGE)"
            ;;
        -h|--help)
            echo "Usage: $0 [--nvidia]"
            echo "  --nvidia, --jetson   Build for NVIDIA Jetson (L4T + nvidia runtime)"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            exit 1
            ;;
    esac
done

# Allow Docker container to connect to X server
xhost +local:docker >/dev/null 2>&1 || true

docker compose up --build "$SERVICE"
