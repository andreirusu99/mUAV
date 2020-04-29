# mUAV
Quadcopter control system based on UDP and MultiWii Serial Protocol.

## Overview
This system allows remote control of a quadcopter from a laptop or another Linux computer by using a common joystick connected to it.

The system's main components are the ground station computer and the companion computer on the quadcopter.
The ground computer reads input from a common joystick connected via USB and sends them via an UDP server to the companion computer onboard the aircraft.

The data is processed and interpreted to perform various command on the vehicle.
