#!/usr/bin/env bash

set -e

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

IMAGE_NAME="robomaster_bridge:latest"
SERVICE_NAME="robomaster_bridge"

if [ -z "$(docker images -q ${IMAGE_NAME} 2> /dev/null)" ]; then
    echo "Build docker container"
    docker build . -t ${IMAGE_NAME}
fi

echo "Prepare system service and config files"

set +e
read -r -d '' bridge_service <<'EOF'
[Unit]
Description=RoboMaster CAN Bridge
After=docker.service systemd-networkd.service network-online.target
Requires=docker.service systemd-networkd.service
Wants=network-online.target

[Service]
Type=exec
# Bring up SocketCAN if the interface already exists
#ExecStartPre=/bin/sh -c '/sbin/ip link show can0 >/dev/null 2>&1'
#ExecStartPre=/bin/sh -c '/sbin/ip link set can0 up type can bitrate 1000000'
ExecStartPre=/bin/sh -c '/sbin/ip link set can0 txqueuelen 65536'
ExecStart=/usr/bin/docker run --rm \
  --name robomaster_bridge \
  --network host \
  --privileged \
  --hostname %H \
  robomaster_bridge:latest \
  /bin/bash -lc "source /opt/ros/humble/setup.bash && source /opt/robomaster_ws/install/setup.bash && ros2 launch src/robomaster_ros2_can/robomaster_can_ros_bridge/launch/bridge.launch.py"
ExecStop=/usr/bin/docker stop -t 10 robomaster_bridge
KillSignal=SIGINT
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

read -r -d '' can_modules <<'EOF'
can
can_raw
mcp251x
EOF

read -r -d '' can_network <<'EOF'
[Match]
Name=can0

[CAN]
BitRate=1M
RestartSec=100ms
EOF
set -e

echo "Write systemd service"
echo "${bridge_service}" > /etc/systemd/system/${SERVICE_NAME}.service

echo "Write kernel module load config"
echo "${can_modules}" > /etc/modules-load.d/can.conf

echo "Write systemd-networkd CAN config"
mkdir -p /etc/systemd/network
echo "${can_network}" > /etc/systemd/network/80-can.network

echo "Reload systemd"
systemctl daemon-reload

echo "Enable systemd-networkd to bring up CAN on boot"
systemctl enable systemd-networkd
systemctl restart systemd-networkd

echo "Enable and start RoboMaster bridge service"
systemctl enable ${SERVICE_NAME}
systemctl restart ${SERVICE_NAME}

echo "Done."
echo "Make sure your Pi boot config has the correct MCP2515 overlay for your exact Waveshare HAT revision, then reboot if you changed boot config."
