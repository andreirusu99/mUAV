#!/usr/bin/env python

import time, threading
from Modules.pyMultiwii import MultiWii
import Modules.UDPserver as udp

axes = {'YAW' : 0, 'THROTTLE' : 1, 'ROLL' : 3, 'PITCH' : 4, 'LTRIG' : 2, 'RTRIG' : 5}
buttons = {'A' : 0, 'B' : 1, 'X' : 2, 'Y' : 3, 'LS' : 4, 'RS' : 5, 'BACK' : 6, 'START' : 7, 'XBOX' : 8, 'LDOWN' : 9, 'RDOWN' : 10}
hat = {'CENTER' : (0,0), 'LEFT' : (-1,0), 'RIGHT' : (1,0), 'UP' : (0,1), 'DOWN' : (0,-1)}


# RC commnads to be sent to the Flight Controller 
# Order: roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4
rcCMD = [1500,1500,1500,1000,1000,1000,1000,1000]

# MRUAV initialization
#vehicle = MultiWii("/dev/tty.usbserial-A801WZA1")
vehicle = MultiWii("/dev/ttyUSB0")

# Function to update commands and attitude to be called by a thread
# def sendCommands():
#     global vehicle, rcCMD
#     try:
#         while True:
#             if udp.active:
#                 current = time.time()
#                 elapsed = 0
#                 # Part for applying commands to the vehicle.
#                 # Joystick manual commands
#                 rcCMD[0] = udp.message[0] # Roll
#                 rcCMD[1] = udp.message[1] # Pitch
#                 rcCMD[2] = udp.message[2] # Yaw
#                 rcCMD[3] = udp.message[3] # Throttle
#                 vehicle.sendCMD(16,MultiWii.SET_RAW_RC,rcCMD)
#                 print(udp.message) # Just to display some data, not really needed.
#                 # 100hz loop
#                 while elapsed < 0.01:
#                     elapsed = time.time() - current
#                 # End of the main loop
#     except (Exception,error):
#         print("Error on sendCommands thread: " + str(error))
#         sendCommands()

# if __name__ == "__main__":
#     try:
#         vehicleThread = threading.Thread(target=sendCommands)
#         vehicleThread.daemon=True
#         vehicleThread.start()
#         udp.startTwisted()

#     except (Exception, error):
#         print("Error on main: " + str(error))
#         vehicle.ser.close()

#     except KeyboardInterrupt:
#         print("Keyboard Interrupt, exiting.")
#         exit()