#!/usr/bin/env python3

"""
Command Dispatcher

Communicates directly with the Flight Controller via MultiWii Serial Protocol.
-> Obtains Attitude information and passes it along to the other components of the system.
-> Passes through commands to the FC in order to control the drone.

"""
from yamspy import MSPy
from Globals import *
import time, threading, os

_TAG = "Dispatcher"

CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 1000,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

FCinfo = ['MSP_ANALOG', 'MSP_ATTITUDE']

print(time.ctime(),_TAG, "Started")

drone = MSPy(device = serialPort, loglevel = "INFO", baudrate = 115200)

drone.is_ser_open = not drone.connect(trials = drone.ser_trials)

if drone.is_ser_open :
    print(time.ctime(), _TAG, "Connected to FC on {}".format(serialPort))
else :
    print(time.ctime(), _TAG, "Error opening serial port.")
    os._exit(1)


def updateCommands(sticks):
    CMDS['roll'] = sticks[0]
    CMDS['pitch'] = sticks[1]
    CMDS['throttle'] = sticks[2]
    CMDS['yaw'] = sticks[3]

def getInfo():
    info = ""
    # get info from FC
    attitude = [0.0] * 3
    for next_msg in FCinfo:

        if drone.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
            dataHandler = drone.receive_msg()
            drone.process_recv_data(dataHandler)

        if next_msg == "MSP_ANALOG":
            voltage = drone.ANALOG['voltage']
            amperage = drone.ANALOG['amperage']
            power = voltage * amperage
            percent = (voltage - 9.9) / 2.73 * 100
            percent = min(max(percent, 0), 100)
            info += "{:.2f}V = {:.1f}% {:.1f}W".format(voltage, percent, power)

        if next_msg == "MSP_ATTITUDE":
            x = drone.SENSOR_DATA['kinematics'][0]
            y = drone.SENSOR_DATA['kinematics'][1]
            z = drone.SENSOR_DATA['kinematics'][2]
            attitude = {'roll': x, 'pitch': y, 'yaw': z}
            info += " R: {:.1f} P: {:.1f} Y: {:.1f}".format(x, y, z)

    return info, attitude

def armDrone():
    CMDS['aux1'] = 2000
    
def disarmDrone():
    CMDS['aux1'] = 1000

def writeToBoard():
    # send the command to the board
    if(drone.send_RAW_RC([CMDS[i] for i in CMDS_ORDER])):
        dataHandler = drone.receive_msg()
        drone.process_recv_data(dataHandler)