#!/usr/bin/env bash

set -e

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

ROBOT_ID="${1:?Usage: sudo $0 <robot_id>  (e.g. sudo $0 4)}"
IMAGE_NAME="robomaster_bridge:latest"
SERVICE_NAME="robomaster_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOCAL_DOMAIN=10
PUBLIC_DOMAIN=0

echo "Ensure Docker is enabled"
systemctl enable docker
systemctl start docker

if [ -z "$(docker images -q ${IMAGE_NAME} 2>/dev/null)" ]; then
    echo "Build docker container"
    MEM_TOTAL_KB=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
    MEM_LIMIT=$(( MEM_TOTAL_KB * 3 / 4 ))k
    docker build --memory="${MEM_LIMIT}" . -t ${IMAGE_NAME}
fi

echo "Write robot/domain config"
cat >/etc/robomaster_bridge.conf <<EOF
ROBOT_ID=${ROBOT_ID}
LOCAL_DOMAIN=${LOCAL_DOMAIN}
PUBLIC_DOMAIN=${PUBLIC_DOMAIN}
EOF

echo "Install Cyclone DDS config"
cp "${SCRIPT_DIR}/../cyclone_dds.xml" /etc/cyclone_dds.xml

echo "Prepare systemd service"

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
ExecStartPre=/usr/bin/systemctl start docker
ExecStartPre=/bin/sh -c 'timeout 30 sh -c "until ip link show can0 >/dev/null 2>&1; do echo Waiting for can0...; sleep 1; done"'
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
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

read -r -d '' bridge_timer <<'EOF'
[Unit]
Description=Start RoboMaster CAN Bridge after boot delay

[Timer]
OnBootSec=30
Unit=robomaster_bridge.service

[Install]
WantedBy=timers.target
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

echo "Write systemd service and timer"
echo "${bridge_service}" > /etc/systemd/system/${SERVICE_NAME}.service
echo "${bridge_timer}" > /etc/systemd/system/${SERVICE_NAME}.timer

echo "Write kernel module load config"
echo "${can_modules}" > /etc/modules-load.d/can.conf

echo "Write systemd-networkd CAN config"
mkdir -p /etc/systemd/network
echo "${can_network}" > /etc/systemd/network/80-can.network

echo "Reload systemd"
systemctl daemon-reload

echo "Enable systemd-networkd"
systemctl enable systemd-networkd
systemctl restart systemd-networkd

echo "Use timer for delayed bridge startup"
systemctl disable ${SERVICE_NAME}.service || true
systemctl enable ${SERVICE_NAME}.timer
systemctl restart ${SERVICE_NAME}.timer

echo "Start bridge now"
systemctl restart ${SERVICE_NAME}.service

echo "Done."
echo ""
echo "Topology:"
echo "  Bridge container runs on ROS_DOMAIN_ID=${PUBLIC_DOMAIN}"
echo ""
echo "Topics exposed by the bridge:"
echo "  /robot_${ROBOT_ID}/cmd_vel       (publish — velocity commands)"
echo "  /robot_${ROBOT_ID}/cmd_wheels    (publish — wheel commands)"
echo "  /robot_${ROBOT_ID}/battery_state (subscribe — battery telemetry)"
echo ""
echo "Check status with:"
echo "  systemctl status ${SERVICE_NAME}.service --no-pager -l"
echo "  systemctl status ${SERVICE_NAME}.timer --no-pager"
echo "  journalctl -u ${SERVICE_NAME}.service -b --no-pager"
