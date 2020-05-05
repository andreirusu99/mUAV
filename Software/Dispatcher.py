#!/usr/bin/env python3

"""
Command Dispatcher

Communicates directly with the Flight Controller via MultiWii Serial Protocol.
-> Obtains Attitude information and passes it along to the other components of the system.
-> Passes through commands from the Command Router to the FC in order to control the _drone.
-> Implements logic for manually overriding the Pilot
"""
import time, threading, os
from Modules.pyMultiwii import MultiWii

_TAG = "Dispatcher"

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

_manualCmd = [1500, 1500, 1500, 1000]
_pilotCmd = [1500, 1500, 1500, 1000]

# accessed externally
mode = 'manual'

#_serialPort = "/dev/tty.usbserial-A801WZA1"
_serialPort = "/dev/ttyUSB0"

_drone = MultiWii(_serialPort)

# called by the Interceptor
def submitManualControl(joy_input):
    global _manualCmd 
    _manualCmd = joy_input
    print(_TAG + " Received: ", _manualCmd)

# called by the Pilot
def submitPilotControl(pilot_input):
    global _pilotCmd
    _pilotCmd = pilot_input

def writeToFlightController():
    global _drone, _drone_cmd
    try:
        while True:

            print(_TAG, _drone_cmd)

            if mode == 'manual':
                _drone.sendCMD(16, MultiWii.SET_RAW_RC, _manualCmd)   

            elif mode == 'auto':
                _drone.sendCMD(16, MultiWii.SET_RAW_RC, _pilotCmd)   

            # 100hz loop
            while elapsed < update_rate:
                elapsed = time.time() - current
            # End of the main loop

    except Exception as error:
        print(_TAG
            + " ERROR on writeToFlightController thread: " 
            + str(error))

        writeToFlightController()


if __name__ == "__main__":
    try:
        # Start the forwarding thread
        dispatcherThread = threading.Thread(target = writeToFlightController)
        dispatcherThread.daemon = True
        dispatcherThread.start()

    except Exception as error:
        print(_TAG
            + " ERROR on main: " 
            + str(error))
        _drone.ser.close()
        os._exit(1)

    except KeyboardInterrupt:
        print(_TAG
            + " Exitting...")
        os._exit(1)
