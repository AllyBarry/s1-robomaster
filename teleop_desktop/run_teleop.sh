#!/bin/bash
# Launch the teleop GUI over X11 forwarding.
# Assumes you've SSH'd in with `ssh -X` or `ssh -Y` to the machine running this.
#
# Usage:
#   ./run_teleop.sh             # x86/standard host
#   ./run_teleop.sh --nvidia    # NVIDIA Jetson host (uses L4T base + nvidia runtime)

set -e

SERVICE="teleop"

for arg in "$@"; do
    case "$arg" in
        --nvidia|--jetson)
            SERVICE="teleop-jetson"
            echo "Running in NVIDIA Jetson mode (runtime: nvidia)"
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
