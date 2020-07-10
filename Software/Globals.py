#!/usr/bin/env python3

# Defining the globally accesible variables
# and run-time parameters

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

safe_message = [0.0,0.0,0.0,-1.0,-1.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

mode = 'manual'

armed = False

serialPort = "/dev/ttyACM2"

STICK_MIN = 1450
STICK_MAX = 1550 