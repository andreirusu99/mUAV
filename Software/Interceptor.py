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
from Modules.pyMultiwii import MultiWii
import Modules.UDPserver as udp
from Modules.utils import axis, button, hat, mapping, clear

import Dispatcher as dispatch
import Pilot as pilot

_TAG = "Interceptor"

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

# telementry info to be relayed to the gorund station
_telemetry = []

def processInput(udp_message):
    # PWM conversion
    roll     = int(mapping(udp_message[0], -1.0, 1.0, 1000, 2000))
    pitch    = int(mapping(udp_message[1], 1.0, -1.0, 1000, 2000))
    yaw      = int(mapping(udp_message[2], -1.0, 1.0, 1000, 2000))
    throttle = int(mapping(udp_message[3],0, -1.0, 1000, 2000))
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
    dead_zone = 100
    if abs(roll - 1500) < dead_zone: roll = 1500
    if abs(pitch - 1500) < dead_zone: pitch = 1500
    if abs(yaw - 1500) < dead_zone: yaw = 1500
    if abs(throttle - 1000) < 50: throttle = 1000

    return [roll, pitch, yaw, throttle, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]


def interceptAndForwardCommands():
    try:
        while True:
            current = time.time()
            elapsed = 0
            
            if udp.active:

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

                #print(_TAG, dispatch.mode, triggers)

                # Switch between auto and manual modes
                if A == 1:
                    dispatch.mode = 'auto'
                elif B == 1:
                    dispatch.mode = 'manual'

                # arm or disarm the drone
                if triggers[1] == 2000:
                    dispatch.armDrone()

                if triggers[0] == 2000:
                    dispatch.disarmDrone()
                
                if dispatch.mode == 'manual' and dispatch.armed:
                    # Send manual control to the Dispatcher
                    dispatch.submitManualControl(control_axes)
                ###

            else:
                # Send safe command to the Dispatcher
                dispatch.submitManualControl([1500, 1500, 1500, 1000])

            while elapsed < update_rate:
                    elapsed = time.time() - current

    except Exception as error:
        print(_TAG
            + ": ERROR on interceptAndForwardCommands thread: " 
            + str(error))
        #interceptAndForwardCommands()


if __name__ == "__main__":

    try:

        # Start the intercepting thread
        interceptorThread = threading.Thread(target = interceptAndForwardCommands)
        interceptorThread.daemon = True
        interceptorThread.start()

        udp.startTwisted()

    except Exception as error:
        print(_TAG
            + ": ERROR on main: " 
            + str(error))
        os._exit(1)

    except KeyboardInterrupt:
        print(_TAG
            + ": Exitting...")
        os._exit(1)