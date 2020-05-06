#!/usr/bin/env python3

"""
Drone Pilot

The brain of the flight control system.
-> Controls the movement of the drone in space
-> Reads sensor information through interfaces
-> Implements motion algorithms to control the drone
-> Implements routines for common command types (hold position, return home, land etc.)

"""

import time, threading, os

import Dispatcher as dispatch

_TAG = "Pilot"

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

# lat lng
_gps = ()

# alt
_baro = 0

# low alt
_sonar = 0

# camera angle
_servo = 60

# drone attitude
_attitude = []

# called by the Interceptor
def getTelemetry():
    # assemble a telemetry packet from 
    # the sensor readings and the drone attitude
    pass

def updateTelemetry():
    global _gps, _baro, _sonar, _attitude
    try:
        while True:
            current = time.time()
            elapsed = 0

            # read the sensor interfaces
            # and store the values in the global variables

            # update attitude
            _attitude = dispatch.attitude

            # 100hz loop
            while elapsed < update_rate:
                elapsed = time.time() - current
            # End of the main loop

    except Exception as error:
        print(_TAG
            + " ERROR on updateTelemetry thread: " 
            + str(error))

        updateTelemetry()

if __name__ == "__main__":
    try:
        # Start the telemetry thread
        telemetryThread = threading.Thread(target = updateTelemetry)
        telemetryThread.daemon = True
        telemetryThread.start()

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