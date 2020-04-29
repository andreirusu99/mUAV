#!/usr/bin/env python3

"""
Command Dispatcher

Communicates directly with the Flight Controller via MultiWii Serial Protocol.
-> Obtains Attitude information and passes it along to the other components of the system.
-> Passes through commands from the Command Router to the FC in order to control the drone.

"""
import time, threading, os
from Modules.pyMultiwii import MultiWii

TAG = "Dispatcher"

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

drone_cmd = [1500, 1500, 1500, 1000]

#serial_port = "/dev/tty.usbserial-A801WZA1"
serial_port = "/dev/ttyUSB0"

drone = MultiWii(serial_port)

# Receive input from the Router
def updateInput(control_cmd):
    drone_cmd = control_cmd

def writeToFlightController():
    global drone, drone_cmd
    try:
        while True:

            print(TAG, drone_cmd)

            drone.sendCMD(16, MultiWii.SET_RAW_RC, drone_cmd)   

            # 100hz loop
            while elapsed < update_rate:
                elapsed = time.time() - current
            # End of the main loop

    except Exception as error:
        print(TAG
            + " ERROR on writeToFlightController thread: " 
            + str(error))

        writeToFlightController()


if __name__ == "__main__":
    try:
        # Start the forwarding thread
        droneThread = threading.Thread(target = writeToFlightController)
        droneThread.daemon = True
        droneThread.start()

    except Exception as error:
        print(TAG
            + " ERROR on main: " 
            + str(error))
        drone.ser.close()
        os._exit(1)

    except KeyboardInterrupt:
        print(TAG
            + " Exitting...")
        os._exit(1)
