#!/usr/bin/env python3

""" 
Command Interceptor (Air)

The entry and exit point of data into and from the drone.
Acts as a bridge between the GCS and the air vehicle.

-> Reads the data coming from the GCS over UDP
-> Maps Joystick axis values to PWM values
-> Filters and applies other transformations to the values (dead-zone etc.)
-> Constructs commands from the received data (button presses etc.)
-> Passes the new information forward to the Command Router (RC control + commands)

"""

import time, threading
from Modules.pyMultiwii import MultiWii
import Modules.UDPserver as udp
from Modules.utils import axis, button, hat, mapping
import Router as router

# The PWM-converted joystick input
joy_input = [1000, 1000, 1000, 1000]

# Function to update commands and attitude to be called by a thread
def interceptCommands():
    
    try:
        while True:
            global joy_input
            if udp.active:
                current = time.time()
                elapsed = 0

                roll     = int(mapping(udp.message[0],-1.0,1.0,1000,2000))
                pitch    = int(mapping(udp.message[1],1.0,-1.0,1000,2000))
                yaw      = int(mapping(udp.message[2],-1.0,1.0,1000,2000))
                throttle = int(mapping(udp.message[3],1.0,-1.0,1000,2000))

                # Joystick manual input from Ground Station
                joy_input = [roll, pitch, yaw, throttle]

                print(joy_input) # Just to display some data, not really needed.

                # 100hz loop
                while elapsed < 0.01:
                    elapsed = time.time() - current
                # End of the main loop

    except Exception as error:
        print("ERROR on interceptCommands thread: " + str(error))
        interceptCommands()

def forwardCommands():
    try:
        while True:
            # Send the data to the Command Router
            router.updateManualInput(joy_input)

            # 100hz loop
            while elapsed < 0.01:
                elapsed = time.time() - current
            # End of the main loop

    except Exception as error:
        print("ERROR on forwardCommands thread: " + str(error))
        interceptCommands()

if __name__ == "__main__":
    try:
        # Start the intercepting thread
        interceptorThread = threading.Thread(target = interceptCommands)
        interceptorThread.daemon=True
        interceptorThread.start()

        # Start the forwarding thread
        # forwarderThread = threading.Thread(target = forwardCommands)
        # forwarderThread.daemon=True
        # forwarderThread.start()

        udp.startTwisted()

    except Exception as error:
        print("ERROR on main: " + str(error))

    except KeyboardInterrupt:
        print("EXITING...")
        exit()