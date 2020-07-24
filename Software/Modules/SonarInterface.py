#!/usr/bin/env python3

"""
Sonar Interface

Interface which triggers and reads a HC-SR04 ultrasonic sensor, mounted on the bottom of the vehicle
to obtain the precise altitude for low-altitude flying (up to 4m above ground).

"""
#Libraries
import Jetson.GPIO as GPIO
import time
 
#set GPIO Pins
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
 
def getDistance():
    try:
        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
         
        #set GPIO direction (IN / OUT)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        return reading()

    finally:
        GPIO.cleanup()
