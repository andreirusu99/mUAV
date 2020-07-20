#!/usr/bin/env python3

"""
Sonar Interface

Interface which triggers and reads a HC-SR04 ultrasonic sensor, mounted on the bottom of the vehicle
to obtain the precise altitude for low-altitude flying (up to 4m above ground).

"""
#Libraries
import Jetson.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
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
 
    return distance
 
if __name__ == '__main__':
    try:
        while True:
            dist = distance()
            if dist > 0.1:
                print("Measured Distance = {:.1f}cm".format(dist))
            time.sleep(0.01)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
