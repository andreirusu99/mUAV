#!/usr/bin/env python3

import rospy
import time
import threading
import os

from drone.msg import ControlAxes as ControlAxesMsg
from drone.msg import Attitude as AttitudeMsg
from std_msgs.msg import Float32 as SonarMsg
from std_msgs.msg import Float64
from std_msgs.msg import Bool

AUTO_TAKEOFF_DELAY = 2

CMDS = {
    'roll':     1500,
    'pitch':    1500,
    'throttle': 1000,
    'yaw':      1500
}

CMDS_MASK = {
    'roll':     True,
    'pitch':    True,
    'throttle': True,
    'yaw':      True
}

height = 0.0


def handleManualRPYT(data):

    CMDS['roll'] = data.axis[0]
    CMDS['pitch'] = data.axis[1]
    CMDS['throttle'] = data.axis[2] if CMDS_MASK['throttle'] else CMDS['throttle']
    CMDS['yaw'] = data.axis[3]


def handleSonar(data):
    global height
    height = data.data


def handlePIDThrottle(data):
    throttle = round(data.data)
    # rospy.logwarn("{}: PID THROTTLE: {}".format(rospy.get_caller_id(), throttle))


def main():

    rospy.init_node('Pilot')

    rospy.Subscriber('ManualControl', ControlAxesMsg, handleManualRPYT, queue_size=1)
    rospy.Subscriber('SonarReading', SonarMsg, handleSonar, queue_size=1)
    rospy.Subscriber('control_effort', Float64, handlePIDThrottle, queue_size=1)

    control_pub = rospy.Publisher('Control', ControlAxesMsg, queue_size=1)
    height_pub = rospy.Publisher('state', Float64, queue_size=1)
    set_height_pub = rospy.Publisher('setpoint', Float64, queue_size=1)
    pid_enable_pub = rospy.Publisher('pid_enable', Bool, queue_size=1)

    global height
    takeoff_time = time.time()
    active = False

    start_time = time.time()
    while not rospy.is_shutdown():
        armed = rospy.get_param("/run/armed")
        engaged = rospy.get_param("/pilot/auto")

        pid_enable_pub.publish(Bool(True))
        set_height_pub.publish(Float64(12.0))  # cm

        height_pub.publish(height)

        if armed and engaged:


            if not active and time.time() - start_time >= AUTO_TAKEOFF_DELAY:
                rospy.logwarn("{}: TAKING OFF...".format(
                    rospy.get_caller_id()))
                CMDS_MASK['throttle'] = False
                active = True
                takeoff_time = time.time()

        else:
            # regain manual control
            CMDS_MASK['throttle'] = True
            active = False
            pid_enable_pub.publish(False)

        # send the final commands to the Dispatcher
        control_pub.publish(ControlAxesMsg([CMDS['roll'],
                                            CMDS['pitch'],
                                            CMDS['throttle'],
                                            CMDS['yaw']]))


if __name__ == "__main__":

    try:

        main()

    except rospy.ROSInterruptException as error:
        pass
