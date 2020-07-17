#!/bin/bash

# configures the Jetson Nano as an access point
sudo hostapd -B /etc/hostapd/hostapd.conf

# with a fixed ip address
sudo ifconfig wlan0 192.168.10.1

# and a dhcp server
sudo service isc-dhcp-server restart