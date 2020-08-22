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
<<<<<<< HEAD
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
=======
GPIO_TRIGGER = 27
GPIO_ECHO = 17
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # set Trigger to HIGH
    # print("TRIG: HIGH")
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    # print("TRIG: LOW")
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        #print("ECHO: LOW")
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = TimeElapsed * 17150
>>>>>>> 77f3d471aa74d2e5b4149c59aeac432ba7f94a4f
 
    return distance
 
def getDistance():
    try:
<<<<<<< HEAD
        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
         
        #set GPIO direction (IN / OUT)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        return reading()

    finally:
=======
        while True:
            dist = distance()
            if dist > 0.1:
                print("Measured Distance = {:.1f}cm".format(dist))
            time.sleep(0.01)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
>>>>>>> 77f3d471aa74d2e5b4149c59aeac432ba7f94a4f
        GPIO.cleanup()
