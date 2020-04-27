#!/usr/bin/env python3

import struct, time, socket
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

UDPport = 51001
message = [0.0,0.0,0.0,-1.0]
active = False

def timeout():
    global active, message
    if not active:
        # There is no UDP data, so give message "safe" commands
        message = [0.0,0.0,0.0,-1.0]
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
    reactor.run()