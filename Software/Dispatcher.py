#!/usr/bin/env python3

"""
Command Dispatcher

Communicates directly with the Flight Controller via MultiWii Serial Protocol.
-> Obtains Attitude information and passes it along to the other components of the system.
-> Passes through commands to the FC in order to control the drone.

"""
import time, threading, os
from Globals import *
from Modules.pyMultiwii import MultiWii
from Modules.utils import clear

_TAG = "Dispatcher"

_manualCmd = [1500, 1500, 1500, 1000]
_pilotCmd = [1500, 1500, 1500, 1000]

_drone = MultiWii(serialPort)

# roll, pitch, heading
attitude = [0, 0, 0]

# called by the Interceptor
def submitManualControl(joy_input):
    global _manualCmd 
    _manualCmd = joy_input

# called by the Pilot
def submitPilotControl(pilot_input):
    global _pilotCmd
    _pilotCmd = pilot_input

def armDrone():
    if not armed:
        _drone.getData(MultiWii.RC)

        roll = _drone.rcChannels['roll']
        pitch = _drone.rcChannels['pitch']
        yaw = _drone.rcChannels['yaw']
        throttle = _drone.rcChannels['throttle']

        if [roll, pitch, yaw, throttle] == [1500, 1500, 1500, 1000]:
            print(_TAG + ": Arming...")
            _drone.arm()
            armed = True

def disarmDrone():
    if armed:
        _drone.getData(MultiWii.RC)

        roll = _drone.rcChannels['roll']
        pitch = _drone.rcChannels['pitch']
        yaw = _drone.rcChannels['yaw']
        throttle = _drone.rcChannels['throttle']

        if [roll, pitch, yaw, throttle] == [1500, 1500, 1500, 1000]:
            # safe to disarm
            print(_TAG + ": Disarming...")
            _drone.disarm()
            armed = False

def writeToFCgetAttitude():
    global attitude
    try:
        while True:
            current = time.time()
            elapsed = 0

            if armed:
                # print(_TAG, ": Mode: " + mode, _manualCmd)
                # clear()

                # write command to FC if armed
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

            while elapsed < update_rate:
                elapsed = time.time() - current

    except Exception as error:
        print(_TAG
            + ": ERROR on writeToFCgetAttitude thread: " 
            + str(error))

        writeToFCgetAttitude()

# Start the dispatcher thread
dispatcherThread = threading.Thread(target = writeToFCgetAttitude)
dispatcherThread.daemon = True
dispatcherThread.start()
