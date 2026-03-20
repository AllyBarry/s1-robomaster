#!/usr/bin/env bash

set -e

if ! ip link show can0 >/dev/null 2>&1; then
  echo "can0 not found. Check MCP2515 overlay / Waveshare HAT setup on the Raspberry Pi host."
  exit 1
fi

if ! ip link show can0 | grep -q "UP"; then
  sudo ip link set can0 up type can bitrate 1000000 berr-reporting on restart-ms 100
fi

sudo ip link set can0 txqueuelen 65536

docker run --rm \
  --network host \
  --privileged \
  --hostname "$(cat /etc/hostname)" \
  robomaster_bridge:latest \
  /bin/bash -lc "source /opt/ros/humble/setup.bash && source /opt/robomaster_ws/install/setup.bash && ros2 launch src/robomaster_ros2_can/robomaster_can_ros_bridge/launch/bridge.launch.py"
