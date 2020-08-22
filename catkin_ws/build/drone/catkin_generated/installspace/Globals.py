#!/usr/bin/env python3

# Defining the globally accesible variables
# and run-time parameters

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

safe_message = [0.0,0.0,-1.0,0.0,-1.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

modes = {'manual':0, "altitude_hold":1}
mode = 0

armed = False

serialPort = "/dev/ttyACM0"

# height PID's gains
h_gains = {'kp': 1, 'ki':1, 'kd':1, 'iMax':2, 'filter_bandwidth':50}

g = 9.81 # m/s2

UAV_MASS = 1.2 # Kg
u0 = 1000 # Zero throttle command
uh = 1300 # Hover throttle command
kt = UAV_MASS * g / (uh-u0)

TIMEOUT_TH = 1
SONAR_RING = 10
VARIANCE_TH = 2000

STICK_MIN = 1200
STICK_MAX = 1800
THROTTLE_MAX = 1700
ROLL_TRIM = 4
PITCH_TRIM = 1
YAW_TRIM = 0
