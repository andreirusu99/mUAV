#!/usr/bin/env python3

"""
Command Dispatcher

Communicates directly with the Flight Controller via MultiWii Serial Protocol.
-> Obtains Attitude information and passes it along to the other components of the system.
-> Passes through commands to the FC in order to control the drone.

"""
from yamspy import MSPy
import time, threading, os
from Globals import armed, update_rate, mode, CMDS_ORDER, serialPort
from Modules.utils import clear

_TAG = "Dispatcher"
            
manualCMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1000
            }

pilotCMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1000
            }

# roll, pitch, heading
attitude = [0, 0, 0]

# called by the Interceptor
def submitManualControl(joy_input):
    global manualCMDS

    manualCMDS['roll'] = joy_input[0]
    manualCMDS['pitch'] = joy_input[1]
    manualCMDS['throttle'] = joy_input[2]
    manualCMDS['yaw'] = joy_input[3]

    #print(_TAG + ": Received: ", manualCMDS)

# called by the Pilot
def submitPilotControl(pilot_input):
    global pilotCMDS
    pilotCMDS = pilot_input

def armDrone():
    global armed
    if not armed:
        print(_TAG + ": Arming...")
        manualCMDS['aux1'] = 1800 # armed
        armed = True

def disarmDrone():
    global armed
    if armed:
        print(_TAG + ": Disarming...")
        manualCMDS['aux1'] = 1000 # disarmed
        armed = False

def writeToFCgetAttitude():
    global attitude

    try:

        with MSPy(device = serialPort, loglevel = "WARNING", baudrate = 115200) as _drone:
            print("Device opened on {}".format(serialPort))

            while True:
                current = time.time()
                elapsed = 0

                if armed:
                    # print(_TAG, ": Mode: " + mode, manualCMDS)
                    # clear()

                    # write command to FC if armed
                    if mode == 'manual':
                        if _drone.send_RAW_RC([manualCMDS[i] for i in CMDS_ORDER]):
                              dataHandler = _drone.receive_msg()
                              _drone.process_recv_data(dataHandler)

                    elif mode == 'auto':
                         if _drone.send_RAW_RC([pilotCMDS[i] for i in CMDS_ORDER]):
                              dataHandler = _drone.receive_msg()
                              _drone.process_recv_data(dataHandler) 

                    # update drone attitude
                    #_drone.getData(MultiWii.ATTITUDE)

                # attitude = [
                #     _drone.attitude['angx'],
                #     _drone.attitude['angy'],
                #     _drone.attitude['heading']
                #     ]

                #print(_TAG + " Attitude: ", attitude)

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
