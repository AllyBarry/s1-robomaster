#!/usr/bin/env bash
# Installs a lightweight health monitor that logs CPU, memory, and temperature
# to journald every 30 seconds. View with: journalctl -u pi-health-monitor -f

set -e

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

MONITOR_SCRIPT="/usr/local/bin/pi-health-monitor.sh"
SERVICE_NAME="pi-health-monitor"

echo "Install health monitor script"
cat > "${MONITOR_SCRIPT}" <<'SCRIPT'
#!/bin/bash
while true; do
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    MEM_TOTAL=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
    MEM_AVAIL=$(awk '/MemAvailable/ {print $2}' /proc/meminfo)
    MEM_USED_PCT=$(awk "BEGIN {printf \"%.1f\", 100 * (1 - ${MEM_AVAIL}/${MEM_TOTAL})}")
    CPU_LOAD=$(awk '{print $1}' /proc/loadavg)
    TEMP="N/A"
    if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
        TEMP_RAW=$(cat /sys/class/thermal/thermal_zone0/temp)
        TEMP=$(awk "BEGIN {printf \"%.1f\", ${TEMP_RAW}/1000}")C
    fi
    DOCKER_MEM="0kB"
    if command -v docker &> /dev/null; then
        DOCKER_MEM=$(docker stats --no-stream --format "{{.MemUsage}}" 2>/dev/null | paste -sd, - || echo "unavailable")
    fi

    echo "${TIMESTAMP} | load: ${CPU_LOAD} | mem: ${MEM_USED_PCT}% used (${MEM_AVAIL}kB avail) | temp: ${TEMP} | docker: [${DOCKER_MEM}]"

    # Warn if memory is critically low (< 100MB available)
    if [ "${MEM_AVAIL}" -lt 102400 ] 2>/dev/null; then
        echo "WARNING: Memory critically low — ${MEM_AVAIL}kB available"
    fi

    sleep 30
done
SCRIPT
chmod +x "${MONITOR_SCRIPT}"

echo "Install systemd service"
cat > "/etc/systemd/system/${SERVICE_NAME}.service" <<EOF
[Unit]
Description=Raspberry Pi Health Monitor
After=multi-user.target

[Service]
Type=simple
ExecStart=${MONITOR_SCRIPT}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl start "${SERVICE_NAME}"

echo "Health monitor installed and running."
echo "View logs with: journalctl -u ${SERVICE_NAME} -f"
