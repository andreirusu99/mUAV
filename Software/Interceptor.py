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
    roll     = int(mapping(udp_message[0],-1.0,1.0,1000,2000))
    pitch    = int(mapping(udp_message[1],1.0,-1.0,1000,2000))
    yaw      = int(mapping(udp_message[2],-1.0,1.0,1000,2000))
    throttle = int(mapping(udp_message[3],1.0,-1.0,1000,2000))
    LT = int(mapping(udp_message[4],1.0,-1.0,1000,2000))
    RT = int(mapping(udp_message[5],1.0,-1.0,1000,2000))

    A = int(udp_message[6])
    B = int(udp_message[7])
    X = int(udp_message[8])
    Y = int(udp_message[9])
    LS = int(udp_message[10])
    RS = int(udp_message[11])
    hat_LR, hat_UD = int(udp_message[12]), int(udp_message[13])

    return [roll, pitch, yaw, throttle, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]


# Function to update commands and attitude to be called by a thread
def interceptAndForwardCommands():
    try:
        while True:
            
            if udp.active:
                current = time.time()
                elapsed = 0

                joystick = processInput(udp.message)

                # Joystick manual input from Ground Station
                control_axes = joystick[:4]

                triggers = joystick[5:6]

                buttons = joystick[7:10]

                shoulders = joystick[11:12]

                hat = joystick[13:14]

                # Switch between auto and manual modes
                if dispatch.mode == 'manual':
                    if A == 1:
                        dispatch.mode = 'auto'
                    else:
                        # Send manual control to the Dispatcher
                        dispatch.submitManualControl(control_axes)

                elif dispatch.mode == 'auto':
                    if B == 1:
                        dispatch.mode = 'manual'
                ###

                # 100hz loop
                while elapsed < update_rate:
                    elapsed = time.time() - current
                # End of the main loop

    except Exception as error:
        print(_TAG
            + " ERROR on interceptAndForwardCommands thread: " 
            + str(error))

        interceptAndForwardCommands()

if __name__ == "__main__":
    try:
        # Start the intercepting thread
        interceptorThread = threading.Thread(target = interceptAndForwardCommands)
        interceptorThread.daemon = True
        interceptorThread.start()

        udp.startTwisted()

    except Exception as error:
        print(_TAG
            + " ERROR on main: " 
            + str(error))
        os._exit(1)

    except KeyboardInterrupt:
        print(_TAG
            + " Exitting...")
        os._exit(1)