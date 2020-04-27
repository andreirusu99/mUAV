#!/usr/bin/env python3

"""
Command Router (Air)

The central hub of the Air side.
-> Handles pass-through of data from the Coammand Interceptor (manual input) and the Pilot (auto).
-> Through-puts data to the Command Dispatcher, which then directly talks to the Flight Controller.
-> Implements logic for manually overriding the Pilot

"""