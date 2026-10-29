#!/bin/bash
# Linux + NetworkManager: bring the OS1 link up after an unplug or power cut, then wait for the sensor.
# Uses a dedicated link-local Ethernet profile that never autoconnects, so it can't take over the port
# from other uses. Create it once (replace enp1s0 with your Ethernet interface):
#   sudo nmcli con add type ethernet ifname enp1s0 con-name os1-link autoconnect no \
#        ipv4.method link-local ipv6.method link-local
# Usage: scripts/os1_up.sh <sensor-host-or-ip> [profile-name]
set -u
SENSOR=${1:?usage: os1_up.sh <sensor-host-or-ip> [profile-name]}
PROFILE=${2:-os1-link}
sudo nmcli con up "$PROFILE" >/dev/null && echo "$PROFILE activated"
for i in $(seq 1 15); do
  if ping -c 1 -W 2 "$SENSOR" >/dev/null 2>&1; then echo "OS1 $SENSOR is up"; exit 0; fi
  echo "waiting for OS1 to boot... ($((i * 5)) s)"; sleep 5
done
echo "OS1 not answering after 75 s: check its power and the Ethernet cable"; exit 1
