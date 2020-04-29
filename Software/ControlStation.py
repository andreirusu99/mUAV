#!/usr/bin/env python2

"""
Ground Control Station (GCS)

Reads a joystick connected to the computer and sends the axis and button values
over UDP to the aircraft computer.

"""

import sys, os, imp
home_dir = os.path.expanduser("~")
utils_file = os.path.join(home_dir, "Desktop/mUAV/Software/Modules/utils.py")
udp_file = os.path.join(home_dir, "Desktop/mUAV/Software/Modules/UDPserver.py")

# Load the utils module using imp
utils = imp.load_source('utils', utils_file)

# Load the udp module using imp
udp = imp.load_source('udp', udp_file)

import pygame
import socket, struct, time

TAG = "Control Station"

# Main configuration
#UDP_IP = "127.0.0.1" # Localhost (for testing)
UDP_IP = "127.0.0.1" # Vehicle IP address
UDP_PORT = 51001 # This port matches the ones using on other scripts

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

# Create UDP socket
sockt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

except Exception as error:
    print(TAG
        + " ERROR: No joystick connected on the computer, " 
        + str(error))
    exit()

while True:
    current = time.time()
    elapsed = 0
    
    # Joystick reading
    pygame.event.pump()

    roll     = round(joystick.get_axis(utils.axis['ROLL']), 3)
    pitch    = round(joystick.get_axis(utils.axis['PITCH']), 3)
    yaw      = round(joystick.get_axis(utils.axis['YAW']), 3)
    throttle = round(joystick.get_axis(utils.axis['THROTTLE']), 3)
    # LT = round(joystick.get_axis(utils.axis['LTRIG']), 3)
    # RT = round(joystick.get_axis(utils.axis['RTRIG']), 3)

    # A = joystick.get_button(utils.button['A'])
    # B = joystick.get_button(utils.button['B'])
    # X = joystick.get_button(utils.button['X'])
    # Y = joystick.get_button(utils.button['Y'])
    # LS = joystick.get_button(utils.button['LS'])
    # RS = joystick.get_button(utils.button['RS'])
    # hat_LR, hat_UD = joystick.get_hat(0)
    
    message = [
    roll, pitch, yaw, throttle
    # , LT, RT
    # , A, B, X, Y, LS, RS
    # , hat_LR, hat_UD
    ]
    
    # Print message to STDOUT
    print(TAG, message)
    utils.clear()
    
    # Assemble message
    buf = struct.pack('>' + 'd' * len(message), *message)
    
    # Send message via UDP
    sockt.sendto(buf, (UDP_IP, UDP_PORT))
    
    # Make this loop work at update_rate
    while elapsed < update_rate:
        elapsed = time.time() - current