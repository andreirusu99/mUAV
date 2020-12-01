#!/usr/bin/env python

import sys
import threading
import time

import rospy
from drone.msg import ControlAxes as ControlAxesMsg

import UDP_Listener as udp


def mapping(value, iMin, iMax, oMin, oMax):
    return (value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin


def processInput(udp_message):
    # PWM conversion
    yaw_low = 1100
    yaw_high = 1900
    roll = int(mapping(udp_message[0], -1.0, 1.0, 1000, 2000))
    pitch = int(mapping(udp_message[1], 1.0, -1.0, 1000, 2000))
    yaw = int(mapping(udp_message[2], -1.0, 1.0, yaw_low, yaw_high))
    throttle = int(mapping(udp_message[3], 1.0, -1.0, 1000, 2000))
    LT = int(mapping(udp_message[4], 0.0, 1.0, 1000, 2000))
    RT = int(mapping(udp_message[4], -1.0, 1.0, 1000, 2000))

    A = int(udp_message[5])
    B = int(udp_message[6])
    X = int(udp_message[7])
    Y = int(udp_message[8])
    LS = int(udp_message[9])
    RS = int(udp_message[10])
    hat_LR, hat_UD = int(udp_message[11]), int(udp_message[12])

    if throttle < 1000:
        throttle = 1000

    return [roll, pitch, throttle, yaw, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]


def UDPthread():
    try:
        udp.startTwisted()

    except Exception as error:
        udp.stop()
        rospy.logerr("{}: {}".format(rospy.get_caller_id(), error))


def main():
    rospy.init_node('InterceptorNode')

    pub = rospy.Publisher('Control', ControlAxesMsg, queue_size=1)

    arm_time = disarm_time = 0.0
    last_camera = last_print = 0.0
    last_active = 0.0
    rate = rospy.Rate(20)  # Hz
    while not rospy.is_shutdown():

        armed = rospy.get_param("/run/armed")

        if udp.active:
            last_active = time.time()

            joystick = processInput(udp.message)

            joy_sticks = joystick[:4]

            triggers = joystick[4:6]

            A, B, X, Y, shoulders, hat = \
                joystick[6], joystick[7], joystick[8], joystick[9], joystick[10:12], joystick[12:14]

            # manual arming (debounced)
            if triggers[1] > 1800 and joy_sticks[2] <= 1100 and not armed and time.time() - disarm_time >= 1:
                rospy.logwarn("{}: ARMING...".format(rospy.get_caller_id()))
                rospy.set_param("/run/armed", True)
                arm_time = time.time()
            elif triggers[1] > 1800 and joy_sticks[2] > 1100 and not armed:
                rospy.logwarn("{}: Cannot ARM, lower throttle!".format(rospy.get_caller_id()))

            # manual disarming (debounced)
            if triggers[1] > 1800 and joy_sticks[2] <= 1100 and armed and time.time() - arm_time >= 1:
                rospy.logwarn("{}: DISARMING...".format(rospy.get_caller_id()))
                rospy.set_param("/run/armed", False)
                disarm_time = time.time()

            # set the camera angle parameter manually (debounced)
            if hat[1] != 0 and time.time() - last_camera > 0.2:
                camera_angle = rospy.get_param("/physical/camera_angle")
                camera_angle += hat[1] * 30
                camera_angle = min(max(camera_angle, 0), 90)
                rospy.set_param("/physical/camera_angle", camera_angle)
                rospy.loginfo("{}: Camera @ {}deg".format(rospy.get_caller_id(), camera_angle))
                last_camera = time.time()

            # shoulder yaw
            if shoulders[0] == 1:
                joy_sticks[3] = 1300

            elif shoulders[1] == 1:
                joy_sticks[3] = 1800
            else:
                joy_sticks[3] = 1500

            # publish the controls axes if craft is armed
            if armed:
                pub.publish(ControlAxesMsg(joy_sticks))

        else:  # UDP inactive
            timeout_th = rospy.get_param("/udp/timeout_threshold")

            if time.time() - last_print > 1:
                rospy.loginfo("{}: UDP timeout!".format(rospy.get_caller_id()))
                last_print = time.time()

            # signal lost while armed (flying)
            if armed and time.time() - last_active >= timeout_th:
                rospy.set_param("/run/armed", False)
                rospy.logerr("{}: UDP timeout: DISARMED for safety".format(
                    rospy.get_caller_id()))

                # suspend until connection regained
                while not udp.active and time.time():
                    pass
                rospy.info("{}: UDP connection regained!".format(
                    rospy.get_caller_id()))

        rate.sleep()


if __name__ == "__main__":

    try:
        thread_udp = threading.Thread(target=UDPthread)
        thread_udp.daemon = True
        thread_udp.start()

        main()
        sys.exit()

    except rospy.ROSInterruptException as error:
        sys.exit()