# Wifi

For Odroid to auto connect to a Wifi hot-spot edit the
and include the following at the bottom of the `/etc/network/interfaces`
file:

    auto wlan0
    allow-hotplug wlan0
    iface wlan0 inet dhcp
    wireless-essid <WIFI ESSID WITH OR WITHOUT QUOTES>
