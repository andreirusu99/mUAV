#!/usr/bin/env python3

import struct
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task

PORT = 44415
message = [
    0.0
]

active = False


def timeout():
    global active, message
    active = False
    # print("Not active!")


class twistedUDP(DatagramProtocol):

    def datagramReceived(self, data, addr):
        global message, active
        active = True

        numOfValues = len(data) // 8
        mess = struct.unpack('>' + 'd' * numOfValues, data)
        message = [round(element, 3) for element in mess]

        print(message)


def startTwisted():
    l = task.LoopingCall(timeout)
    l.start(0.5)  # Check for disconnection each 0.5 and send neutral commands
    reactor.listenUDP(PORT, twistedUDP())
    print("Listening on port {}".format(PORT))
    reactor.run(installSignalHandlers=False)
