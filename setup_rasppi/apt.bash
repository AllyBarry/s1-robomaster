#!/usr/bin/env bash
set -e

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

# --- Time sync (must come before apt update) ---
# Raspberry Pi has no RTC — clock can be wrong on first boot,
# causing "Release file is not valid yet" errors from apt.
echo "Syncing system clock..."
apt-get -o Acquire::Check-Valid-Until=false update -y
apt-get install -y systemd-timesyncd fake-hwclock
timedatectl set-ntp true

# fake-hwclock persists the time across reboots so it's approximately
# correct on next boot even without network access.
systemctl enable fake-hwclock
fake-hwclock save

# Block boot until NTP has synced time (prevents services from starting
# with wrong time and getting certificate / apt validity errors).
systemctl enable systemd-time-wait-sync.service

# Wait briefly for NTP to adjust the clock
sleep 2
echo "System time: $(date)"

# --- Base packages ---
apt-get update && \
apt-get -y install vim cmake tmux

cd /tmp
# curl docker

sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate


# to get architecture and OS
# uname -m
# cat /etc/os-release

# docker
# remove old/conflicting packages
# for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do
#   sudo apt-get remove -y $pkg
# done

# req packages
# sudo apt-get update
# sudo apt-get install -y ca-certificates curl gnupg

# add docker's official gpg key
# sudo install -m 0755 -d /etc/apt/keyrings
# curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
# sudo chmod a+r /etc/apt/keyrings/docker.gpg

# echo \
#   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
#   $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
#   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# sudo apt-get update
# sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# set up sudoless user
# sudo usermod -aG docker $USER
# newgrp docker

# enable on boot
# sudo systemctl enable docker
# sudo systemctl start docker
# sudo systemctl status docker

# use docker compose not docker-compose

# set up CAN HAT
# sudo su
# wget https://github.com/joan2937/lg/archive/master.zip
# unzip master.zip
# cd lg-master
# sudo make install 

# update according to https://www.waveshare.com/wiki/RS485_CAN_HAT

# sudo reboot
