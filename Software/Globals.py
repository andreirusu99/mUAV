#!/usr/bin/env python3

# Defining the globally accesible variables
# and run-time parameters

cycle_Hz = 100  # 100 hz loop cycle
update_rate = 1 / cycle_Hz

safe_message = [0.0,0.0,0.0,-1.0,-1.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

mode = 'manual'

armed = False

#serialPort = "/dev/tty.usbserial-A801WZA1"
serialPort = "/dev/ttyUSB0"