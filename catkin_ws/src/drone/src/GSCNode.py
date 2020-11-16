#!/usr/bin/env python3

# Ground Station Communication Node
# reads the ROS parameters and topics and
# relays them back to the Ground Station computer via a socket

import rospy
import socket
import struct
import time

UDP_IP = "192.168.137.1"  # WiFi Ground Station IP address
# UDP_IP = "192.168.100.50"  # LAN Ground Station IP address
UDP_PORT = 44415

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

def main():
    rospy.init_node('GSCNode')

    sockt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rate = rospy.Rate(1) # Hz
    while not rospy.is_shutdown():
        message = [round(time.time())]

        # Assemble message
        buf = struct.pack('>' + 'd' * len(message), *message)

        # Send message via UDP
        sockt.sendto(buf, (UDP_IP, UDP_PORT))

        rate.sleep()

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException as error:
        pass
