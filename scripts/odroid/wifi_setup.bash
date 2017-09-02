#!/bin/bash
set -e

cat << EOF /etc/network/interfaces.d
auto wlan0
allow-hotplug wlan0
iface wlan0 inet dhcp
wireless-essid ATL-TOWER
EOF
