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

#_serialPort = "/dev/tty.usbserial-A801WZA1"
_serialPort = "/dev/ttyUSB0"

_drone = MultiWii(_serialPort)

# accessed externally
mode = 'manual'

# roll, pitch, heading
attitude = [0, 0, 0]

# called by the Interceptor
def submitManualControl(joy_input):
    global _manualCmd 
    _manualCmd = joy_input
    print(_TAG + " Received: ", _manualCmd)

# called by the Pilot
def submitPilotControl(pilot_input):
    global _pilotCmd
    _pilotCmd = pilot_input

def writeToFCgetAttitude():
    global _drone, _drone_cmd, attitude
    try:
        while True:
            current = time.time()
            elapsed = 0

            print(_TAG, " Mode: " + dispatch.mode)

            # write command to FC
            if mode == 'manual':
                _drone.sendCMD(8, MultiWii.SET_RAW_RC, _manualCmd)   

            elif mode == 'auto':
                _drone.sendCMD(8, MultiWii.SET_RAW_RC, _pilotCmd)   

            # update drone attitude
            _drone.getData(MultiWii.ATTITUDE)

            attitude = [
                _drone.attitude['angx'],
                _drone.attitude['angy'],
                _drone.attitude['heading']
                ]

            print(_TAG + " Attitude: ", attitude)

            # 100hz loop
            while elapsed < update_rate:
                elapsed = time.time() - current
            # End of the main loop

    except Exception as error:
        print(_TAG
            + " ERROR on writeToFCgetAttitude thread: " 
            + str(error))

        writeToFCgetAttitude()


if __name__ == "__main__":
    try:
        # Start the dispatcher thread
        dispatcherThread = threading.Thread(target = writeToFCgetAttitude)
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
