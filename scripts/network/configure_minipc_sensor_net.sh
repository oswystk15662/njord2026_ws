#!/usr/bin/env bash
# Keep the MiniPC Ethernet on the MID360S static network without a default route.
set -euo pipefail

SENSOR_CONNECTION=${SENSOR_CONNECTION:-"有線接続 1"}
SENSOR_ADDRESS=${SENSOR_ADDRESS:-192.168.1.2/24}

sudo nmcli connection modify "$SENSOR_CONNECTION" \
  ipv4.method manual ipv4.addresses "$SENSOR_ADDRESS" \
  ipv4.gateway "" ipv4.dns "" ipv4.never-default yes
sudo nmcli connection up "$SENSOR_CONNECTION"

ip -4 address show
