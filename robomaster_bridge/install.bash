#!/usr/bin/env bash

set -e

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

if [[ -z "$USER_HOME" ]]; then
    echo "Could not determine user home."
    exit 1
fi

SCRIPT_PATH="${USER_HOME}/bin/canmon.sh"
SERVICE_DIR="${USER_HOME}/.config/systemd/user"
SERVICE_PATH="${SERVICE_DIR}/canmon.service"

echo "Installing user service for ${USER_NAME}"

mkdir -p "${USER_HOME}/bin"
mkdir -p "${SERVICE_DIR}"

if [[ ! -x "${SCRIPT_PATH}" ]]; then
    echo "Warning: ${SCRIPT_PATH} not found or not executable"
    echo "Create it and run: chmod +x ${SCRIPT_PATH}"
fi

cat > "${SERVICE_PATH}" <<EOF
[Unit]
Description=CAN Monitor tmux session
After=default.target

[Service]
Type=oneshot
ExecStart=${SCRIPT_PATH}
RemainAfterExit=yes

[Install]
WantedBy=default.target
EOF

chown -R "${USER_NAME}:${USER_NAME}" "${USER_HOME}/.config" "${USER_HOME}/bin"

echo "Enable lingering so user services run at boot"
sudo loginctl enable-linger "${USER_NAME}"

echo "Reload and enable service"
sudo -u "${USER_NAME}" XDG_RUNTIME_DIR="/run/user/$(id -u "$USER_NAME")" systemctl --user daemon-reload
sudo -u "${USER_NAME}" XDG_RUNTIME_DIR="/run/user/$(id -u "$USER_NAME")" systemctl --user enable canmon.service
sudo -u "${USER_NAME}" XDG_RUNTIME_DIR="/run/user/$(id -u "$USER_NAME")" systemctl --user restart canmon.service

echo ""
echo "Installed:"
echo "  ${SERVICE_PATH}"
echo ""
echo "Commands:"
echo "  sudo -u ${USER_NAME} systemctl --user status canmon.service"
echo "  tmux attach -t canmon"
