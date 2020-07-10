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

_TAG = "Control Station"

axis = {'YAW' : 0, 'THROTTLE' : 1, 'ROLL' : 3, 'PITCH' : 4, 'LTRIG' : 2, 'RTRIG' : 5}
button = {'A' : 0, 'B' : 1, 'X' : 2, 'Y' : 3, 'LS' : 4, 'RS' : 5, 'BACK' : 6, 'START' : 7, 'XBOX' : 8, 'LDOWN' : 9, 'RDOWN' : 10}
hat = {'CENTER' : (0,0), 'LEFT' : (-1,0), 'RIGHT' : (1,0), 'UP' : (0,1), 'DOWN' : (0,-1)}

# Main configuration
UDP_IP = "127.0.0.1" # Localhost (for testing)
#UDP_IP = "192.168.10.1" # Vehicle IP address
UDP_PORT = 51004 # This port matches the ones using on other scripts

# Create UDP socket
sockt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

try:
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

except Exception as error:
    print(_TAG
        + " ERROR: No joystick connected on the computer, " 
        + str(error))
    exit()

while True:
    current = time.time()
    elapsed = 0
    
    # Joystick reading
    pygame.event.pump()

    roll     = round(joystick.get_axis(axis['ROLL']), 3)
    pitch    = round(joystick.get_axis(axis['PITCH']), 3)
    yaw      = round(joystick.get_axis(axis['YAW']), 3)
    throttle = round(joystick.get_axis(axis['THROTTLE']), 3)
    LT = round(joystick.get_axis(axis['LTRIG']), 3)
    RT = round(joystick.get_axis(axis['RTRIG']), 3)

    A = joystick.get_button(button['A'])
    B = joystick.get_button(button['B'])
    X = joystick.get_button(button['X'])
    Y = joystick.get_button(button['Y'])
    LS = joystick.get_button(button['LS'])
    RS = joystick.get_button(button['RS'])
    hat_LR, hat_UD = joystick.get_hat(0)
    
    message = [
    roll, pitch, yaw, throttle
    , LT, RT
    , A, B, X, Y, LS, RS
    , hat_LR, hat_UD
    ]
    
    # Print message to STDOUT
    print(_TAG, message)
    utils.clear()
    
    # Assemble message
    buf = struct.pack('>' + 'd' * len(message), *message)
    
    # Send message via UDP
    sockt.sendto(buf, (UDP_IP, UDP_PORT))
    
    # Make this loop work at update_rate
    while elapsed < update_rate:
        elapsed = time.time() - current