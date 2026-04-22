#!/bin/bash
# Launch the teleop GUI over X11 forwarding.
# Assumes you've SSH'd in with `ssh -X` or `ssh -Y` to the machine running this.
#
# Works on both x86 and Jetson hosts — the base image is multi-arch, so the
# arm64 variant is pulled automatically on Jetson. No GPU passthrough is
# needed for the teleop GUI itself.

set -e

# Allow Docker container to connect to X server
xhost +local:docker >/dev/null 2>&1 || true

docker compose up --build teleop
