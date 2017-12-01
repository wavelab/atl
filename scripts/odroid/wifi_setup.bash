#!/bin/bash
set -e

cat << EOF >> /etc/network/interfaces

auto lo
iface lo inet loopback

auto wlan0
allow-hotplug wlan0
iface wlan0 inet dhcp
wireless-essid ATL-Tower
EOF

cat << EOF >> /etc/wpa_supplicant/wpa_supplicant.conf

network={
    ssid="ATL-Tower"
    key_mgmt=NONE
}
EOF
