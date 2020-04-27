#!/usr/bin/env python3

"""
Command Dispatcher

Communicates directly with the Flight Controller via MultiWii Serial Protocol.
-> Obtains Attitude information and passes it along to the other components of the system.
-> Passes through commands from the Command Router to the FC in order to control the drone.

"""

from Modules.pyMultiwii import MultiWii