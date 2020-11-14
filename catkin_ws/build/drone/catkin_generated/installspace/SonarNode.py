#!/usr/bin/env python3

import rospy
import time
import threading
import os
import numpy as np

from std_msgs.msg import Float32 as SonarMsg

import Jetson.GPIO as GPIO

TRIG = 27
ECHO = 17


def reading():
    GPIO.output(TRIG, False)
    time.sleep(0.01)

    # set Trigger to HIGH
    GPIO.output(TRIG, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00002)
    GPIO.output(TRIG, False)

    pulse_start = time.time()

    maxTime = 0.02
    timeout = pulse_start + maxTime
    timed_out = False

    while GPIO.input(ECHO) == 0 and pulse_start < timeout:
        pulse_start = time.time()

    if pulse_start >= timeout:
        return 0.0

    pulse_end = time.time()
    timeout = pulse_end + maxTime

    while GPIO.input(ECHO) == 1 and pulse_end < timeout:
        pulse_end = time.time()

    if pulse_end >= timeout:
        return 0.0

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150

    return distance


def main():
    rospy.init_node('SonarNode')

    pub = rospy.Publisher('SonarReading', SonarMsg, queue_size=1)

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # set GPIO direction (IN / OUT)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    it = 0
    ring = 10
    distances = [0.0] * ring
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        it += 1
        it %= ring
        distances[it] = reading()
        median = np.median(distances)
        mean = np.mean(distances)
        distance = np.round((mean + median) / 2, decimals=0)

        rospy.loginfo("{}: {}cm".format(rospy.get_caller_id(), distance))
        pub.publish(SonarMsg(distance))
        rate.sleep()


if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException as error:
        pass
