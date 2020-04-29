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
import Router as router

TAG = "Interceptor"

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

# The PWM-converted joystick input
joy_input = [1500, 1500, 1500, 1000]

# Function to update commands and attitude to be called by a thread
def interceptCommands():
    global joy_input
    try:
        while True:
            
            if udp.active:
                current = time.time()
                elapsed = 0

                print(TAG, udp.message)
                clear()

                roll     = int(mapping(udp.message[0],-1.0,1.0,1000,2000))
                pitch    = int(mapping(udp.message[1],1.0,-1.0,1000,2000))
                yaw      = int(mapping(udp.message[2],-1.0,1.0,1000,2000))
                throttle = int(mapping(udp.message[3],1.0,-1.0,1000,2000))
                # LT = int(mapping(udp.message[4],1.0,-1.0,1000,2000))
                # RT = int(mapping(udp.message[5],1.0,-1.0,1000,2000))

                # A = int(udp.message[6])
                # B = int(udp.message[7])
                # X = int(udp.message[8])
                # Y = int(udp.message[9])
                # LS = int(udp.message[10])
                # RS = int(udp.message[11])
                # hat_LR, hat_UD = int(udp.message[12]), int(udp.message[13])

                # Joystick manual input from Ground Station
                joy_input = [roll, pitch, yaw, throttle]


                # 100hz loop
                while elapsed < update_rate:
                    elapsed = time.time() - current
                # End of the main loop

    except Exception as error:
        print(TAG
            + " ERROR on interceptCommands thread: " 
            + str(error))

        interceptCommands()

def forwardCommands():
    global joy_input
    try:
        while True:
            if udp.active:
                current = time.time()
                elapsed = 0

                # Send the data to the Command Router
                router.updateManualInput(joy_input)

                # 100hz loop
                while elapsed < update_rate:
                    elapsed = time.time() - current
                # End of the main loop

    except Exception as error:
        print(TAG
            + " ERROR on forwardCommands thread: " 
            + str(error))

        forwardCommands()

if __name__ == "__main__":
    try:
        # Start the intercepting thread
        interceptorThread = threading.Thread(target = interceptCommands)
        interceptorThread.daemon = True
        interceptorThread.start()

        # Start the forwarding thread
        forwarderThread = threading.Thread(target = forwardCommands)
        forwarderThread.daemon = True
        forwarderThread.start()

        udp.startTwisted()

    except Exception as error:
        print(TAG
            + " ERROR on main: " 
            + str(error))
        os._exit(1)

    except KeyboardInterrupt:
        print(TAG
            + " Exitting...")
        os._exit(1)