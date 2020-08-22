#!/bin/bash

# configures the Jetson Nano as an access point
sudo hostapd -B /etc/hostapd/hostapd.conf

# with a fixed ip address
sudo ifconfig wlan0 192.168.100.40

# and a dhcp server
sudo service isc-dhcp-server restart
