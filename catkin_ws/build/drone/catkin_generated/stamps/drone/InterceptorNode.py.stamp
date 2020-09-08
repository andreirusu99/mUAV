#!/usr/bin/env python3

import rospy
import time, threading, os

import UDPserver as udp

from drone.msg import ControlAxes as ControlAxesMsg


def mapping(value,iMin,iMax,oMin,oMax):
    return ((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin)


def processInput(udp_message):

    STICK_MIN = rospy.get_param("/stick/limits/min")
    STICK_MAX = rospy.get_param("/stick/limits/max")
    THROTTLE_MAX = rospy.get_param("/stick/limits/throttle_max")

    ROLL_TRIM = rospy.get_param("/stick/trims/roll")
    PITCH_TRIM = rospy.get_param("/stick/trims/pitch")
    YAW_TRIM = rospy.get_param("/stick/trims/yaw")

    # PWM conversion
    yaw_low = STICK_MIN + 100
    yaw_high = STICK_MAX - 100
    pitch     = int(mapping(udp_message[0], 1.0, -1.0, STICK_MIN, STICK_MAX))
    roll      = int(mapping(udp_message[1], -1.0, 1.0, STICK_MIN, STICK_MAX))
    yaw       = int(mapping(udp_message[2], -1.0, 1.0, yaw_low, yaw_high))
    throttle  = int(mapping(udp_message[3], 0, -1.0, 1000, THROTTLE_MAX))
    LT        = int(mapping(udp_message[4], 0.0, 1.0, 1000, 2000))
    RT        = int(mapping(udp_message[4], -1.0, 0.0, 2000, 1000))

    A = int(udp_message[5])
    B = int(udp_message[6])
    X = int(udp_message[7])
    Y = int(udp_message[8])
    LS = int(udp_message[9])
    RS = int(udp_message[10])
    hat_LR, hat_UD = int(udp_message[11]), int(udp_message[12])

    if throttle < 1000: throttle = 1000

    # deadzone configuration
    dead_zone_ratio = 0.1 # 10%
    input_range = STICK_MAX - STICK_MIN
    dead_zone = input_range * dead_zone_ratio

    if abs(roll - 1500) < dead_zone: roll = 1500
    if abs(pitch - 1500) < dead_zone: pitch = 1500
    if abs(yaw - 1500) < dead_zone * 1.5: yaw = 1500
    if abs(throttle - 1000) < 50: throttle = 1000

    roll += ROLL_TRIM
    pitch += PITCH_TRIM
    yaw += YAW_TRIM
    
    return [roll, pitch, throttle, yaw, LT, RT, A, B, X, Y, LS, RS, hat_LR, hat_UD]


def UDPthread():
    _TAG = "UDP Thread"
    
    try:

        udp.startTwisted()
    
    except Exception as error:
        rospy.logerr("{}: {}".format(_TAG, error))
        UDPthread()


def main():

    rospy.init_node('InterceptorNode')

    pub = rospy.Publisher('Control', ControlAxesMsg, queue_size=1)
    
    arm_time = disarm_time = 0.0
    last_camera = 0.0

    while not rospy.is_shutdown():
        
        armed = rospy.get_param("/run/armed")

        if udp.active:
            rospy.set_param("/udp/last_active", time.time())

            joystick = processInput(udp.message)

            joy_sticks = joystick[:4]

            triggers = joystick[4:6]

            A = joystick[6]
            B = joystick[7]
            X = joystick[8]
            Y = joystick[9]

            shoulders = joystick[10:12]

            hat = joystick[12:14]

            # manual arming (debounced)
            if triggers[1] > 1800 and not armed and time.time() - disarm_time >= 1:
                rospy.logwarn("{}: ARMING...".format(rospy.get_caller_id()))
                rospy.set_param("/run/armed", True)
                arm_time = time.time()

            # manual disarming (debounced)
            elif triggers[1] > 1800 and joy_sticks[2] == 1000 and armed and time.time() - arm_time >= 1:
                rospy.logwarn("{}: DISARMING...".format(rospy.get_caller_id()))
                rospy.set_param("/run/armed", False)
                disarm_time = time.time()
            
            # set the camera angle parameter manually (debounced)
            camera_angle = rospy.get_param("/physical/camera_angle")
            if hat[1] != 0 and time.time() - last_camera > 0.2:
                camera_angle += hat[1] * 30
                camera_angle = min(max(camera_angle, 0), 90)
                rospy.set_param("/physical/camera_angle", camera_angle)
                rospy.loginfo("{}: Camera @ {}deg".format(rospy.get_caller_id(), camera_angle))
                last_camera = time.time()

            # publish the control axes if craft is armed
            if armed:
                pub.publish(ControlAxesMsg(joy_sticks))

        else:
            rospy.logwarn("{}: UDP inactive!".format(rospy.get_caller_id()))
            last_active = rospy.get_param("/udp/last_active")
            timeout_th = rospy.get_param("/udp/timeout_threshold")

            # signal lost while armed (flying)
            if armed and time.time() - last_active >= timeout_th:
                rospy.set_param("/run/armed", False)
                rospy.logerr("{}: UDP timeout: DISARM for safety".format(rospy.get_caller_id()))
                
                # suspend until connection regained
                while not udp.active:
                    pass
                rospy.logwarn("{}: UDP connection regained!".format(rospy.get_caller_id()))



if __name__ == "__main__":

    try:
        thread_udp = threading.Thread(target = UDPthread)
        thread_udp.daemon = True
        thread_udp.start()

        main()
        
    except rospy.ROSInterruptException as error:
        pass