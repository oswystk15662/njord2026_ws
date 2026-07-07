#!/bin/bash
# Revert enP8p1s0 back to DHCP after Livox Mid360 verification.
set -euo pipefail

IFACE="enP8p1s0"
CON_NAME="livox-mid360"

if nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
    sudo nmcli connection modify "$CON_NAME" ipv4.method auto
    sudo nmcli connection up "$CON_NAME"
else
    echo "No '$CON_NAME' connection profile found; nothing to revert." >&2
fi

echo "== $IFACE now: =="
ip -4 addr show "$IFACE"
