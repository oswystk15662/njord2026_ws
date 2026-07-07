#!/bin/bash
# Configure enP8p1s0 with a static IPv4 address for talking to the Livox Mid360.
# The Mid360 host-side IP is fixed at 192.168.1.5 in
# src/driver/lidar/livox_ros_driver2/config/MID360_config.json, so this must match.
set -euo pipefail

IFACE="enP8p1s0"
CON_NAME="livox-mid360"
STATIC_IP="192.168.1.5/24"

if ! nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
    sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$CON_NAME" \
        ipv4.method manual ipv4.addresses "$STATIC_IP"
else
    sudo nmcli connection modify "$CON_NAME" ipv4.method manual ipv4.addresses "$STATIC_IP"
fi

sudo nmcli connection up "$CON_NAME"

echo "== $IFACE now: =="
ip -4 addr show "$IFACE"
