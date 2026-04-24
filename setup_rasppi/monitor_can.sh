#!/usr/bin/env bash

SESSION="canmon"
CONTAINER="robomaster_bridge"

USER_NAME="${SUDO_USER:-rasppiuser}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"
TOOLS_DIR="${USER_HOME}/Documents/dars/s1-robomaster/setup_rasppi"

BASHRC="${USER_HOME}/.bashrc"

if ! grep -Fq "${TOOLS_DIR}" "${BASHRC}"; then
    echo "" >> "${BASHRC}"
    echo "# RoboMaster tools path" >> "${BASHRC}"
    echo "export PATH=\"${TOOLS_DIR}:\$PATH\"" >> "${BASHRC}"
fi

chown "${USER_NAME}:${USER_NAME}" "${BASHRC}"

tmux has-session -t "$SESSION" 2>/dev/null && exec tmux attach -t "$SESSION"

# Window 0: host monitoring
tmux new-session -d -s "$SESSION" -n monitor

# Pane 0: service logs
tmux send-keys -t "$SESSION":0.0 \
'journalctl -u robomaster_bridge.service -f' C-m

# Pane 1: CAN stats
tmux split-window -h -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.1 \
'watch -n 1 "ip -s link show can0"' C-m

# Pane 2: CAN details
tmux split-window -v -t "$SESSION":0.1
tmux send-keys -t "$SESSION":0.2 \
'watch -n 1 "ip -details link show can0"' C-m

tmux select-layout -t "$SESSION":0 tiled

# Window 1: raw CAN bus
tmux new-window -t "$SESSION" -n candump
tmux send-keys -t "$SESSION":1 \
'candump -tz can0' C-m

# Window 2: docker shell inside bridge container
tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION":2 \
'docker exec -it robomaster_bridge bash' C-m

# Start on monitor window
tmux select-window -t "$SESSION":0
exec tmux attach -t "$SESSION"
