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
from yamspy import MSPy
from Globals import mode, update_rate, serialPort, CMDS_ORDER, STICK_MIN, STICK_MAX, armed
from Modules import UDPserver as udp
from Modules.utils import mapping, clear
from itertools import cycle

# import Dispatcher as dispatch
# import Pilot as pilot

_TAG = "Interceptor"

# telementry info to be relayed to the ground station
_telemetry = []

drone = MSPy(device = serialPort, loglevel = "WARNING", baudrate = 115200)

CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

FCinfo = cycle(['MSP_ANALOG', 'MSP_ATTITUDE'])

def processInput(udp_message):
    # PWM conversion
    roll     = int(mapping(udp_message[0], -1.0, 1.0, STICK_MIN, STICK_MAX))
    pitch    = int(mapping(udp_message[1], 1.0, -1.0, STICK_MIN, STICK_MAX))
    yaw      = int(mapping(udp_message[2], -1.0, 1.0, STICK_MIN, STICK_MAX))
    throttle = int(mapping(udp_message[3],0, -1.0, 1000, 2000))
    LT = int(mapping(udp_message[4], -1.0, 1.0, 1000, 2000))
    RT = int(mapping(udp_message[5], -1.0, 1.0, 1000, 2000))

    A = int(udp_message[6])
    B = int(udp_message[7])
    X = int(udp_message[8])
    Y = int(udp_message[9])
    LS = int(udp_message[10])
    RS = int(udp_message[11])
    hat_LR, hat_UD = int(udp_message[12]), int(udp_message[13])

    if throttle < 1000: throttle = 1000

    # deadzone configuration
    dead_zone_ratio = 0.1 # 10%
    input_range = STICK_MAX - STICK_MIN
    dead_zone = input_range * dead_zone_ratio

    if abs(roll - 1500) < dead_zone: roll = 1500
    if abs(pitch - 1500) < dead_zone: pitch = 1500
    if abs(yaw - 1500) < dead_zone: yaw = 1500
    if abs(throttle - 1000) < 50: throttle = 1000

    return [roll, pitch, throttle, yaw, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]


def interceptAndForwardCommands():
    global mode, update_rate, CMDS_ORDER, CMDS, armed
    try:
        arm_time = disarm_time = 0
        ready1 = ready2 = False
        printable = ""

        while True:
            current = time.time()
            elapsed = 0

            if udp.active:

                joystick = processInput(udp.message)

                CMDS['roll'] = joystick[0]
                CMDS['pitch'] = joystick[1]
                CMDS['throttle'] = joystick[2]
                CMDS['yaw'] = joystick[3]

                #print(CMDS)

                # Joystick manual input from Ground Station
                control_axes = joystick[:4]

                triggers = joystick[4:6]

                A = joystick[6]
                B = joystick[7]
                X = joystick[8]
                Y = joystick[9]

                shoulders = joystick[10:12]

                hat = joystick[12:14]

                #print(_TAG, mode, joystick)

                # Switch between auto and manual modes
                if A == 1:
                    mode = 'auto'
                elif B == 1:
                    mode = 'manual'

                # arm or disarm the drone
                if triggers[1] > 1800 and not armed and time.time() - disarm_time >= 1:
                    print("Arming...")
                    CMDS['aux1'] = 2000
                    armed = True
                    arm_time = time.time()
                    #print(arm_time)
                                                       # at least 1 second passed from the arming     
                elif triggers[1] > 1800 and armed and time.time() - arm_time >= 1:
                    print("Disarming...")
                    CMDS['aux1'] = 1000
                    armed = False
                    disarm_time = time.time()

                #print(str(armed))
                
            else: # udp not active
                CMDS['aux1'] = 1000
            
            # send the command to the board
            if(drone.send_RAW_RC([CMDS[i] for i in CMDS_ORDER])):
                    dataHandler = drone.receive_msg()
                    drone.process_recv_data(dataHandler)


            # get info from FC
            next_msg = next(FCinfo) # circular list
            
            if drone.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                dataHandler = drone.receive_msg()
                drone.process_recv_data(dataHandler)

            if next_msg == "MSP_ANALOG":
                voltage = drone.ANALOG['voltage']
                amperage = drone.ANALOG['amperage']
                power = voltage * amperage
                printable += "{:.2f}V * {:.2f}A = {:.1f}W".format(voltage, amperage, power)
                ready1 = True

            if next_msg == "MSP_ATTITUDE":
                x = drone.SENSOR_DATA['kinematics'][0]
                y = drone.SENSOR_DATA['kinematics'][1]
                z = drone.SENSOR_DATA['kinematics'][2]
                printable += "\nRoll: {:.1f} Pitch: {:.1f} Yaw: {:.1f}".format(x, y, z)
                ready2 = True

            if ready1 and ready2:
                print(printable)
                ready1 = ready2 = False
                printable = ""

            while elapsed < update_rate:
                    elapsed = time.time() - current

    except Exception as error:
        print(_TAG
            + ": ERROR on Interceptor thread: " 
            + str(error))
        interceptAndForwardCommands()


if __name__ == "__main__":

    try:

        drone.is_ser_open = not drone.connect(trials = drone.ser_trials)
        if drone.is_ser_open :
            print("Connected to FC on {}".format(serialPort))
        else :
            print("Error opening serial port.")
            os._exit(1)

        # Start the intercepting thread
        interceptorThread = threading.Thread(target = interceptAndForwardCommands)
        interceptorThread.daemon = True
        interceptorThread.start()

        udp.startTwisted()

    except Exception as error:
        print(_TAG
            + ": ERROR on main: " 
            + str(error))
        os._exit(1)

    except KeyboardInterrupt:
        print(_TAG
            + ": Exitting...")
        os._exit(1)