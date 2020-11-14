#!/usr/bin/env python3

import rospy
import time
import threading
import os

from drone.msg import ControlAxes as ControlAxesMsg
from drone.msg import Attitude as AttitudeMsg

# [Roll, Pitch, Throttle, Yaw]
CMDS = []

def handleManualRPYT(data):
    CMDS[0] = data.axis[0]
    CMDS[1] = data.axis[1]
    CMDS[2] = data.axis[2]
    CMDS[3] = data.axis[3]


def main():

    rospy.init_node('PilotNode')

    rospy.Subscriber('ManualControl', ControlAxesMsg, handleManualRPYT, queue_size=1)

    pub = rospy.Publisher('Control', ControlAxesMsg, queue_size=1)

    while not rospy.is_shutdown():
        # perform autonomous tasks here, that will influence the commands array

        if len(CMDS) == 4:
            # send the final commands to the Dispatcher
            pub.publish(ControlAxesMsg(CMDS))



if __name__ == "__main__":

    try:

        main()

    except rospy.ROSInterruptException as error:
        pass
