#!/usr/bin/env python3
import rospy

import time, threading, os
import serial
from geometry_msgs.msg import Pose2D as GPSMsg

fix_quality = 0
gps_time = "" # UTC
latitude = 0.0 # N
longitude = 0.0 # E
gps_altitude = 0.0 # m
ground_speed = 0.0 # km/h

def readString(ser):
    while True:
        while ser.read().decode("utf-8") != '$':  # Wait for the begging of the string
            pass  # Do nothing
        line = ser.readline().decode("utf-8")  # Read the entire string
        return line


def getTime(string, format, returnFormat):
    global gps_time
    gps_time = time.strftime(returnFormat, time.strptime(string, format))  # Convert date and time to a nice printable format


def getLatLng(latString, lngString):
    latString = latString[:2].lstrip('0') + "." + "%.7s" % str(float(latString[2:]) * 1.0 / 60.0).lstrip("0.")
    lngString = lngString[:3].lstrip('0') + "." + "%.7s" % str(float(lngString[3:]) * 1.0 / 60.0).lstrip("0.")
    return latString, lngString


def processRMC(lines):
    global latitude, longitude
    getTime(lines[1] + lines[9], "%H%M%S.%f%d%m%y", "%a %b %d %H:%M:%S %Y")
    latlng = getLatLng(lines[3], lines[5])
    latitude = latlng[0]
    longitude = latlng[1]


def processGGA(lines):
    global latitude, longitude, fix_quality, gps_altitude
    getTime(lines[1], "%H%M%S.%f", "%H:%M:%S")
    latlng = getLatLng(lines[2], lines[4])
    latitude = latlng[0]
    longitude = latlng[1]
    fix_quality = lines[6]
    gps_altitude = lines[9]


def processGLL(lines):
    global latitude, longitude
    getTime(lines[5], "%H%M%S.%f", "%H:%M:%S")
    latlng = getLatLng(lines[1], lines[3])
    latitude = latlng[0]
    longitude = latlng[1]


def processVTG(lines):
    global ground_speed
    ground_speed = lines[7]

def checksum(line):
    checkString = line.partition("*")
    checksum = 0
    for c in checkString[0]:
        checksum ^= ord(c)

    try:  # Just to make sure
        inputChecksum = int(checkString[2].rstrip(), 16)
    except:
        rospy.logerr(rospy.get_caller_id(), "Error in string")
        return False

    if checksum == inputChecksum:
        return True
    else:
        rospy.logerr(rospy.get_caller_id(), "Checksum error!")
        return False

def main(ser):

    rospy.init_node('GPSNode')
    gps_pub = rospy.Publisher('Location', GPSMsg, queue_size=1)

    while not rospy.is_shutdown():

        line = readString(ser)
        lines = line.split(",")

        if checksum(line):
            if lines[0] == "GPRMC":
                processRMC(lines)
                
            elif lines[0] == "GPGGA":
                processGGA(lines)
                
            elif lines[0] == "GPGLL":
                processGLL(lines)
                
            elif lines[0] == "GPVTG":
                processVTG(lines)
                
            rospy.loginfo("{}: {}UTC: {:.5f}N {:.5f}E @ {:.1f}M, {:.1f}km/h".format(
                rospy.get_caller_id(),
                gps_time, 
                float(latitude), 
                float(longitude), 
                float(gps_altitude),
                float(ground_speed)))        


if __name__ == '__main__':

    try:
        
        serial_port = '/dev/' + rospy.get_param("/serial/gps_vk")
        ser = serial.Serial(serial_port, 9600, timeout=1)

        main(ser)

    except rospy.ROSInterruptException as error:
        pass