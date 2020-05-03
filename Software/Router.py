#!/usr/bin/env python3

"""
Command Router (Air)

The central hub of the Air side.
-> Handles pass-through of data from the Coammand Interceptor (manual input) and the Pilot (auto).
-> Through-puts data to and from the Command Dispatcher, which then directly talks to the Flight Controller.
-> Implements logic for manually overriding the Pilot

"""

import time, threading, os
import Dispatcher as dispatcher

TAG = "Router"

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

manual_cmd = [1500, 1500, 1500, 1000]
pilot_cmd = [1500, 1500, 1500, 1000]

def updateManualInput(joy_input):
    manual_cmd = joy_input
    print(TAG + "Received: ", manual_cmd)


def sendControlCommands():
    global manual_cmd, pilot_cmd
    try:
        while True:
            current = time.time()
            elapsed = 0

            # send the input to the Dispatcher
            dispatcher.updateInput(manual_cmd)

            # 100hz loop
            while elapsed < update_rate:
                elapsed = time.time() - current
            # End of the main loop

    except Exception as error:
        print(TAG
            + " ERROR on sendControlCommands thread: " 
            + str(error))

        sendControlCommands()


if __name__ == "__main__":
    try:
        # Start the forwarding thread
        senderThread = threading.Thread(target = sendControlCommands)
        senderThread.daemon = True
        senderThread.start()

    except Exception as error:
        print(TAG
            + " ERROR on main: " 
            + str(error))
        os._exit(1)

    except KeyboardInterrupt:
        print(TAG
            + " Exitting...")
        os._exit(1)