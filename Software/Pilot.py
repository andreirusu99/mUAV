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
import numpy as np
from math import *
from Globals import *
from Modules import UDPserver as udp
from Modules.utils import mapping, PID, low_pass, limit

import Dispatcher as dispatch

import Modules.SonarInterface as sonar

_TAG = "Interceptor"

sonar_cycle = [0.0] * SONAR_RING
sonar_reading = 0.0 # updated by the sonar thread

desired_height = 0.0 # updated by the main thread
required_throttle = 1000 # updated by the height PID thread

attitude = {'roll': 0, 'pitch': 0, 'yaw':0}

def processInput(udp_message):
    # PWM conversion
    yaw_low = STICK_MIN + 100
    yaw_high = STICK_MAX - 100
    roll     = int(mapping(udp_message[0], -1.0, 1.0, STICK_MIN, STICK_MAX))
    pitch    = int(mapping(udp_message[1], 1.0, -1.0, STICK_MIN, STICK_MAX))
    yaw      = int(mapping(udp_message[2], -1.0, 1.0, STICK_MIN, STICK_MAX))
    throttle = int(mapping(udp_message[3],0, -1.0, 1000, THROTTLE_MAX))
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
    if abs(yaw - 1500) < dead_zone * 1.5: yaw = 1500
    if abs(throttle - 1000) < 50: throttle = 1000

    roll += ROLL_TRIM
    pitch += PITCH_TRIM
    yaw += YAW_TRIM
    
    return [roll, pitch, throttle, yaw, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]

def sonarThread():
    _TAG = "Sonar Thread"
    global sonar_reading
    try:
        it = 0
        while True:
            it += 1
            it %= 10
            current = time.time()
            elapsed = 0

            # add a sonar reading to the sonar array
            sonar_cycle[it] = sonar.getDistance()

            # the sensor reading is the mean
            # of the last SONAR_RING values
            sonar_reading = np.mean(sonar_cycle)

            variance = np.var(sonar_cycle)
            if variance > VARIANCE_TH:
                sonar_reading = 0.0


            while elapsed < update_rate:
                    elapsed = time.time() - current

    except Exception as error:
        print(time.ctime(), _TAG,
            "ERROR on sonarThread thread: ", 
            str(error))
        sonarThread()

def heightPIDThread():
    _TAG = "Height PID"
    global required_throttle
    hPIDvalue = 0.0
    heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
    f_pitch  = low_pass(20, update_rate)
    f_roll   = low_pass(20, update_rate)

    try:
        it = 0
        while True:
            it += 1
            it %= 10
            current = time.time()
            elapsed = 0

            if mode == modes['altitude_hold'] and desired_height >= 10:

                hPIDvalue = heightPID.update(desired_height - sonar_reading)

                required_throttle = \
                ((hPIDvalue + g) * UAV_MASS) / \
                (cos(f_pitch.update(radians(attitude['pitch'])))
                    * cos(f_roll.update(radians(attitude['roll']))))
                
                required_throttle = (required_throttle / kt) + u0

                required_throttle = limit(required_throttle,1000,2000)

            elif mode == modes['manual']:
                heightPID.resetIntegrator()

            while elapsed < update_rate:
                    elapsed = time.time() - current

    except Exception as error:
        print(time.ctime(), _TAG,
            "ERROR on heightPIDThread thread: ", 
            str(error))
        heightPIDThread()

def mainThread():
    _TAG = "Main Thread"
    global mode, armed, desired_height, required_throttle, sonar_reading
    try:
        arm_time = disarm_time = 0
        last_active = 0
        control_axes = []
        it = 0

        hPIDvalue = 0.0
        heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
        f_pitch  = low_pass(20, update_rate)
        f_roll   = low_pass(20, update_rate)

        while True:
            it += 1
            it %= 10
            current = time.time()
            elapsed = 0

            if udp.active:

                last_active = time.time()

                joystick = processInput(udp.message)

                # Joystick manual input from Ground Station
                control_axes = joystick[:4]

                triggers = joystick[4:6]

                A = joystick[6]
                B = joystick[7]
                X = joystick[8]
                Y = joystick[9]

                shoulders = joystick[10:12]

                hat = joystick[12:14]

                ########### ARMING ###########
                # arm or disarm the drone
                if triggers[1] > 1800 and not armed and time.time() - disarm_time >= 1:
                    print(time.ctime(), _TAG, "ARMING...")
                    armed = True
                    arm_time = time.time()
                    dispatch.armDrone()
                                                       # at least 1 second passed from the arming     
                elif triggers[1] > 1800 and armed and time.time() - arm_time >= 1:
                    print(time.ctime(), _TAG, "DISARMING...")
                    armed = False
                    disarm_time = time.time()
                    desired_height = 0.0
                    required_throttle = 1000
                    mode = modes['manual']
                    dispatch.disarmDrone()

                #################################

                ########### FLIGHT MODE ###########
                # switch between throttle flight modes
                # if armed and mode != modes['manual'] and B == 1:
                #     print(time.ctime(), _TAG, "MANUAL")
                #     desired_height = 0.0
                #     required_throttle = 1000
                #     mode = modes['manual']
                    

                # elif armed and mode != modes['altitude_hold'] and A == 1:
                #     print(time.ctime(), _TAG, "ALT_HOLD")
                #     mode = modes['altitude_hold']

                # mode-specific behaviour
                # if mode == modes['altitude_hold'] and desired_height >= 10:

                #     hPIDvalue = heightPID.update(desired_height - sonar_reading)

                #     required_throttle = \
                #     ((hPIDvalue + g) * UAV_MASS) / \
                #     (cos(f_pitch.update(radians(attitude['pitch'])))
                #         * cos(f_roll.update(radians(attitude['roll']))))
                    
                #     required_throttle = (required_throttle / kt) + u0

                #     required_throttle = limit(required_throttle,1000,THROTTLE_MAX)

                # elif mode == modes['manual']:
                #     heightPID.resetIntegrator()


                #################################

                ########### HEIGHT ###########
                # update the desired height using the hat
                # if hat[1] != 0:
                #     desired_height += hat[1]
                #     desired_height = min(max(desired_height, 0), 300)

                # # replace the stick throttle command with the PID value
                # if mode == modes['altitude_hold']:
                #     control_axes[2] = required_throttle

                # #################################

                # ########### UPDATE SENSORS ###########
                # # add a sonar reading to the sonar array
                # sonar_cycle[it] = sonar.getDistance()

                # # the sensor reading is the mean
                # # of the last SONAR_RING values
                # sonar_reading = np.mean(sonar_cycle)

                # variance = np.var(sonar_cycle)
                # if variance > VARIANCE_TH:
                #     sonar_reading = 0.0

                #################################

                ########### WRITE TO BOARD ###########
                # send control commands to the dispatch
                dispatch.updateCommands(control_axes)
                dispatch.writeToBoard()

                #################################


            else: # udp not active
                print(time.ctime(), _TAG, "UDP timeout")
                # connection lost to Ground Station
                if time.time() - last_active > TIMEOUT_TH:
                    dispatch.disarmDrone()

            board_info = dispatch.getInfo()

            # update attitude information
            attitude = board_info[1]

            ########### PRINTS ###########
            print(time.ctime(), _TAG, board_info[0])
            # print(time.ctime(),  _TAG, "{:.2f}cm now -> {:.2f}cm target".format(sonar_reading, desired_height))
            # print(time.ctime(),  _TAG, "{} now -> {} required".format(control_axes[2], required_throttle))
            
            #################################

            while elapsed < update_rate:
                    elapsed = time.time() - current

    except Exception as error:
        print(time.ctime(), _TAG,
            "ERROR on mainThread: ", 
            str(error))
        mainThread()


if __name__ == "__main__":

    try:
        print(time.ctime(), _TAG, "Started")

        # thread_height = threading.Thread(target = heightPIDThread)
        # thread_height.daemon = True
        # thread_height.start()

        # thread_sonar = threading.Thread(target = sonarThread)
        # thread_sonar.daemon = True
        # thread_sonar.start()

        thread_main = threading.Thread(target = mainThread)
        thread_main.daemon = True
        thread_main.start()

        udp.startTwisted()

    except Exception as error:
        print(time.ctime(), _TAG,
            "ERROR on main: ",
            str(error))
        os._exit(1)

    except KeyboardInterrupt:
        print(time.ctime(), _TAG,
            "Exitting...")
        os._exit(1)
