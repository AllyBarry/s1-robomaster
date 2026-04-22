#!/usr/bin/env bash

set -e

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

ROBOT_ID="${1:?Usage: sudo $0 <robot_id>  (e.g. sudo $0 4)}"
IMAGE_NAME="robomaster_bridge:latest"
SERVICE_NAME="robomaster_bridge"
GATEWAY_SERVICE_NAME="robomaster_domain_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Domain topology:
#   LOCAL_DOMAIN (on-robot, private) — high-rate sensor data stays here
#   PUBLIC_DOMAIN (network) — only bridged topics reach here
LOCAL_DOMAIN=10
PUBLIC_DOMAIN=0

echo "Ensure Docker is enabled on boot"
systemctl enable docker
systemctl start docker

if [ -z "$(docker images -q ${IMAGE_NAME} 2> /dev/null)" ]; then
    echo "Build docker container (limited to 75% RAM to keep SSH alive)"
    MEM_TOTAL_KB=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
    MEM_LIMIT=$(( MEM_TOTAL_KB * 3 / 4 ))k
    docker build --memory="${MEM_LIMIT}" . -t ${IMAGE_NAME}
fi

echo "Write robot ID config"
echo "ROBOT_ID=${ROBOT_ID}" > /etc/robomaster_bridge.conf

echo "Install Cyclone DDS config"
cp "${SCRIPT_DIR}/../cyclone_dds.xml" /etc/cyclone_dds.xml

# echo "Install domain_bridge config (substitute ROBOT_ID)"
# sed "s/@ROBOT_ID@/${ROBOT_ID}/g" \
#     "${SCRIPT_DIR}/domain_bridge.yaml.template" \
#     > /etc/robomaster_domain_bridge.yaml

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
EnvironmentFile=/etc/robomaster_bridge.conf
# Wait for CAN interface to exist, then ensure it is up
ExecStartPre=/bin/sh -c 'for i in $(seq 1 30); do ip link show can0 >/dev/null 2>&1 && break; echo "Waiting for can0... ($i)"; sleep 1; done'
ExecStartPre=/bin/sh -c '/sbin/ip link set can0 up type can bitrate 1000000 2>/dev/null || true'
ExecStartPre=/bin/sh -c '/sbin/ip link set can0 txqueuelen 65536'
ExecStart=/usr/bin/docker run --rm \
  --name robomaster_bridge \
  --network host \
  --privileged \
  --hostname %H \
  --memory 512m \
  --cpus 2.0 \
  -v /etc/cyclone_dds.xml:/etc/cyclone_dds.xml:ro \
  -e ROBOT_ID=${ROBOT_ID} \
  -e ROS_DOMAIN_ID=${PUBLIC_DOMAIN} \
  -e CYCLONEDDS_URI=/etc/cyclone_dds.xml \
  robomaster_bridge:latest \
  /bin/bash -lc "source /opt/ros/humble/setup.bash && source /opt/robomaster_ws/install/setup.bash && ros2 launch src/robomaster_ros2_can/robomaster_can_ros_bridge/launch/bridge.launch.py"
ExecStop=/usr/bin/docker stop -t 10 robomaster_bridge
KillSignal=SIGINT
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

# Domain bridge gateway is disabled for now — all nodes run on a single
# ROS_DOMAIN_ID. Re-enable this block, the service write, and the
# systemctl enable/restart lines below when re-introducing the gateway.
# read -r -d '' gateway_service <<'EOF'
# [Unit]
# Description=RoboMaster Domain Bridge Gateway
# After=docker.service robomaster_bridge.service
# Requires=docker.service
# Wants=robomaster_bridge.service
#
# [Service]
# Type=exec
# ExecStart=/usr/bin/docker run --rm \
#   --name robomaster_domain_bridge \
#   --network host \
#   --memory 256m \
#   --cpus 1.0 \
#   -v /etc/cyclone_dds.xml:/etc/cyclone_dds.xml:ro \
#   -v /etc/robomaster_domain_bridge.yaml:/etc/bridge.yaml:ro \
#   -e CYCLONEDDS_URI=/etc/cyclone_dds.xml \
#   robomaster_bridge:latest \
#   /bin/bash -lc "source /opt/ros/humble/setup.bash && source /opt/robomaster_ws/install/setup.bash && ros2 run domain_bridge domain_bridge /etc/bridge.yaml"
# ExecStop=/usr/bin/docker stop -t 10 robomaster_domain_bridge
# KillSignal=SIGINT
# Restart=always
# RestartSec=2
#
# [Install]
# WantedBy=multi-user.target
# EOF

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

echo "Write systemd services"
echo "${bridge_service}" > /etc/systemd/system/${SERVICE_NAME}.service
# echo "${gateway_service}" > /etc/systemd/system/${GATEWAY_SERVICE_NAME}.service

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

# echo "Enable and start domain bridge gateway service"
# systemctl enable ${GATEWAY_SERVICE_NAME}
# systemctl restart ${GATEWAY_SERVICE_NAME}

echo "Done."
echo ""
echo "Topology:"
echo "  Bridge nodes run on ROS_DOMAIN_ID=${LOCAL_DOMAIN}"
echo "  (Domain-bridge gateway is disabled — set ROS_DOMAIN_ID=${LOCAL_DOMAIN}"
echo "   on external nodes to talk to the bridge directly.)"
echo ""
echo "Topics exposed by the bridge:"
echo "  /robot_${ROBOT_ID}/cmd_vel      (publish — velocity commands)"
echo "  /robot_${ROBOT_ID}/cmd_wheels   (publish — wheel commands)"
echo "  /robot_${ROBOT_ID}/battery_state (subscribe — battery telemetry)"
echo ""
echo "Make sure your Pi boot config has the correct MCP2515 overlay for your exact Waveshare HAT revision, then reboot if you changed boot config."
