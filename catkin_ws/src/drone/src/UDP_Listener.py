#!/usr/bin/env python3

import struct
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

PORT = 51444
message = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
]

active = False

def timeout():
    global active, message
    active = False
    if not active:
        # There is no UDP data, so give "safe" commands
        message = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, addr):
        # print("FROM {}".format(addr))
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
