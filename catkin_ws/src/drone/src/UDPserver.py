<<<<<<< HEAD
#!/usr/bin/env python3

import rospy
import struct, time, socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

UDPport = 51444
message = [
0.0,0.0,0.0,-1.0 ,-1.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
]

active = False

def timeout():
    global active, message
    if not active:
        # There is no UDP data, so give message "safe" commands
        message = [0.0,0.0,0.0,0.0,-1.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    active = False

class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, addr):
        global message, active
        active = True
        # In case of failure on receiving, check this:
        numOfValues = len(data) // 8
        mess = struct.unpack('>' + 'd' * numOfValues, data)
        message = [ round(element, 3) for element in mess ]

def startTwisted():
    l = task.LoopingCall(timeout)
    l.start(0.5) # Check for disconnection each 0.5 and send neutral commands
    reactor.listenUDP(UDPport, twistedUDP())
    reactor.run(installSignalHandlers=False)
=======
#!/usr/bin/env python3

import rospy
import struct
import time
import socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

PORT = 51444
message = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
]

active = False

def timeout():
    global active, message
    active = False
    if not active:
        # There is no UDP data, so give "safe" commands
        message = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]



class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, addr):
        global message, active
        active = True

        numOfValues = len(data) // 8
        mess = struct.unpack('>' + 'd' * numOfValues, data)
        message = [round(element, 3) for element in mess]


def startTwisted():
    l = task.LoopingCall(timeout)
    l.start(0.5)  # Check for disconnection each 0.5 and send neutral commands
    reactor.listenUDP(PORT, twistedUDP())
    reactor.run(installSignalHandlers=False)
>>>>>>> 1bd9aa140a2cfff867a7dd6752a2e0ff265ed22a
