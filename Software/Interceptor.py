#!/usr/bin/env python3

""" 
Command Interceptor (Air)

The entry and exit point of data into and from the drone.
Acts as a bridge between the GCS and the air vehicle.

-> Reads the data coming from the GCS over UDP
-> Maps Joystick axis values to PWM values
-> Filters and applies other transformations to the values (dead-zone etc.)
-> Constructs commands from the received data (button presses etc.)
-> Passes the new information forward to the Command Router (RC control + commands)

"""

import time, threading
from Modules.pyMultiwii import MultiWii
import Modules.UDPserver as udp
from Modules.utils import axis, button, hat

# Order: roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]

serial_port = "/dev/ttyUSB0"

# MRUAV initialization
#vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")

############
#vehicle = MultiWii(serial_port)

#Function to update commands and attitude to be called by a thread
def sendCommands():
    global vehicle, rcCMD
    try:
        while True:
            if udp.active:
                current = time.time()
                elapsed = 0
                
                # Part for applying commands to the vehicle.
                # Joystick manual commands
                rcCMD[0] = udp.message[0] # Roll
                rcCMD[1] = udp.message[1] # Pitch
                rcCMD[2] = udp.message[2] # Yaw
                rcCMD[3] = udp.message[3] # Throttle

                ##############################
                #vehicle.sendCMD(16, MultiWii.SET_RAW_RC,rcCMD)

                print(udp.message) # Just to display some data, not really needed.
                # 100hz loop
                while elapsed < 0.01:
                    elapsed = time.time() - current
                # End of the main loop
    except Exception as error:
        print("Error on sendCommands thread: " + str(error))
        sendCommands()

if __name__ == "__main__":
    try:
        vehicleThread = threading.Thread(target = sendCommands)
        vehicleThread.daemon=True
        vehicleThread.start()
        udp.startTwisted()

    except Exception as error:
        print("Error on main: " + str(error))
        ###################
        #vehicle.ser.close()

    except KeyboardInterrupt:
        print("Keyboard Interrupt, exiting.")
        exit()