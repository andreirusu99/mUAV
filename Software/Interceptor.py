#!/usr/bin/env python3

""" 
Command Interceptor (Air)

Acts as a bridge between the GCS and the air vehicle.

-> Reads the data coming from the GCS over UDP
-> Maps Joystick axis values to PWM values
-> Filters and applies other transformations to the values (dead-zone etc.)
-> Constructs commands from the received data (button presses etc.)
-> Passes the new information forward to the Command Router (RC control + commands)

"""

import time, threading, os
from Globals import *
from Modules import UDPserver as udp
from Modules.utils import mapping

import Dispatcher as dispatch

# import Dispatcher as dispatch
# import Pilot as pilot

_TAG = "Interceptor"

# telementry info to be relayed to the ground station
_telemetry = []

def processInput(udp_message):
    # PWM conversion
    roll     = int(mapping(udp_message[0], -1.0, 1.0, STICK_MIN, STICK_MAX))
    pitch    = int(mapping(udp_message[1], 1.0, -1.0, STICK_MIN, STICK_MAX))
    yaw      = int(mapping(udp_message[2], -1.0, 1.0, STICK_MIN, STICK_MAX))
    throttle = int(mapping(udp_message[3],0, -1.0, 1000, THROTTLE_MAX))
    LT = int(mapping(udp_message[4], -1.0, 1.0, 1000, 2000))
    RT = int(mapping(udp_message[5], -1.0, 1.0, 1000, 2000))

    A = int(udp_message[6])
    B = int(udp_message[7])
    X = int(udp_message[8])
    Y = int(udp_message[9])
    LS = int(udp_message[10])
    RS = int(udp_message[11])
    hat_LR, hat_UD = int(udp_message[12]), int(udp_message[13])

    if throttle < 1000: throttle = 1000

    # deadzone configuration
    dead_zone_ratio = 0.1 # 10%
    input_range = STICK_MAX - STICK_MIN
    dead_zone = input_range * dead_zone_ratio

    if abs(roll - 1500) < dead_zone: roll = 1500
    if abs(pitch - 1500) < dead_zone: pitch = 1500
    if abs(yaw - 1500) < dead_zone: yaw = 1500
    if abs(throttle - 1000) < 50: throttle = 1000

    roll += ROLL_TRIM
    pitch += PITCH_TRIM
    yaw += YAW_TRIM
    
    return [roll, pitch, throttle, yaw, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]


def interceptAndForwardCommands():
    global mode, update_rate, CMDS_ORDER, CMDS, armed, PITCH_TRIM, ROLL_TRIM
    try:
        arm_time = disarm_time = 0
        ready1 = ready2 = False
        printable = ""
        last_active = 0

        while True:
            current = time.time()
            elapsed = 0

            if udp.active:

                last_active = time.time()

                joystick = processInput(udp.message)

                # Joystick manual input from Ground Station
                control_axes = joystick[:4]

                triggers = joystick[4:6]

                A = joystick[6]
                B = joystick[7]
                X = joystick[8]
                Y = joystick[9]

                shoulders = joystick[10:12]

                hat = joystick[12:14]

                # arm or disarm the drone
                if triggers[1] > 1800 and not armed and time.time() - disarm_time >= 1:
                    print(time.ctime(), _TAG, "ARMING...")
                    armed = True
                    arm_time = time.time()
                    dispatch.armDrone()
                                                       # at least 1 second passed from the arming     
                elif triggers[1] > 1800 and armed and time.time() - arm_time >= 1:
                    print(time.ctime(), _TAG, "DISARMING...")
                    armed = False
                    disarm_time = time.time()
                    dispatch.disarmDrone()

                # send control commands to the dispatch
                dispatch.updateCommands(control_axes)
                dispatch.writeToBoard()

                print(time.ctime(), dispatch.getInfo())

            else: # udp not active
                print(time.ctime(), _TAG, "UDP timeout")
                # connection lost to Ground Station
                if time.time() - last_active > TIMEOUT_TH:
                    dispatch.disarmDrone()

            while elapsed < update_rate:
                    elapsed = time.time() - current

    except Exception as error:
        print(time.ctime(), _TAG,
            "ERROR on Interceptor thread: ", 
            str(error))
        interceptAndForwardCommands()


if __name__ == "__main__":

    try:
        print(time.ctime(), _TAG, "Started")
        # Start the intercepting thread
        interceptorThread = threading.Thread(target = interceptAndForwardCommands)
        interceptorThread.daemon = True
        interceptorThread.start()

        udp.startTwisted()

    except Exception as error:
        drone.conn.close()
        print(time.ctime(), _TAG,
            "ERROR on main: ",
            str(error))
        os._exit(1)

    except KeyboardInterrupt:
        drone.conn.close()
        print(time.ctime(), _TAG,
            "Exitting...")
        os._exit(1)