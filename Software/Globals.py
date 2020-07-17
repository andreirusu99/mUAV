#!/usr/bin/env python3

# Defining the globally accesible variables
# and run-time parameters

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

safe_message = [0.0,0.0,-1.0,0.0,-1.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

mode = 'manual'

armed = False

serialPort = "/dev/ttyACM0"

TIMEOUT_TH = 1

STICK_MIN = 1200
STICK_MAX = 1800
THROTTLE_MAX = 1700
ROLL_TRIM = -15
PITCH_TRIM = 10
YAW_TRIM = 0