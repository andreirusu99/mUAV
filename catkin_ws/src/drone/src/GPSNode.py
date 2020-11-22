#!/usr/bin/env python3

import time

import numpy as np
import rospy
import serial
from drone.msg import GPSinfo as GPSMsg


fix_quality = 0
gps_time = ""  # UTC
latitude = 0.0  # N
longitude = 0.0  # E
elevation = 0.0  # m
ground_speed = 0.0  # km/h


def readString(ser):
    while True:
        while ser.read().decode("utf-8") != '$':  # Wait for the begging of the string
            pass  # Do nothing
        line = ser.readline().decode("utf-8")  # Read the entire string
        return line


def getTime(string, format, returnFormat):
    global gps_time
    # Convert date and time to a nice printable format
    gps_time = time.strftime(returnFormat, time.strptime(string, format))


def getLatLng(latString, lngString):
    if len(latString) > 0 and len(lngString) > 0:
        latString = latString[:2].lstrip(
            '0') + "." + "%.7s" % str(float(latString[2:]) * 1.0 / 60.0).lstrip("0.")
        lngString = lngString[:3].lstrip(
            '0') + "." + "%.7s" % str(float(lngString[3:]) * 1.0 / 60.0).lstrip("0.")
        return latString, lngString


def processRMC(lines):
    global latitude, longitude
    getTime(lines[1] + lines[9], "%H%M%S.%f%d%m%y", "%a %b %d %H:%M:%S %Y")
    latlng = getLatLng(lines[3], lines[5])
    latitude = latlng[0]
    longitude = latlng[1]


def processGGA(lines):
    global latitude, longitude, fix_quality, elevation
    getTime(lines[1], "%H%M%S.%f", "%H:%M:%S")
    latlng = getLatLng(lines[2], lines[4])
    latitude = latlng[0]
    longitude = latlng[1]
    if len(lines[6]) > 0:
        fix_quality = lines[6]
    if len(lines[9]) > 0:
        elevation = lines[9]


def processGLL(lines):
    global latitude, longitude
    getTime(lines[5], "%H%M%S.%f", "%H:%M:%S")
    latlng = getLatLng(lines[1], lines[3])
    latitude = latlng[0]
    longitude = latlng[1]


def processVTG(lines):
    global ground_speed
    if len(lines[7]) > 0:
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
    global latitude, longitude, elevation, ground_speed, fix_quality
    rospy.init_node('GPSNode')

    gps_pub = rospy.Publisher('/GPS', GPSMsg, queue_size=1)

    rate = rospy.Rate(1)
    it = 0
    ring = 5
    elevs = [0.0] * ring
    speeds = [0.0] * ring
    fixed = False
    while not rospy.is_shutdown():

        line = readString(ser)
        lines = line.split(",")

        try:
            if lines[0] == "GPRMC":
                processRMC(lines)

            elif lines[0] == "GPGGA":
                processGGA(lines)

            elif lines[0] == "GPGLL":
                processGLL(lines)

            elif lines[0] == "GPVTG":
                processVTG(lines)

            fix_quality = int(fix_quality)

            if fix_quality > 0:

                if not fixed:
                    rospy.loginfo("{}: Fix gained with {} sat".format(rospy.get_caller_id(), fix_quality))
                    fixed = True

                latitude = float(latitude)
                longitude = float(longitude)
                elevation = round(float(elevation))
                ground_speed = round(float(ground_speed), 1)

                if ground_speed < 10:
                    ground_speed = 0

           
                it += 1
                it %= ring

                elevs[it] = elevation
                speeds[it] = ground_speed

                elevs_sorted = np.sort(elevs)
                speeds_sorted = np.sort(speeds)

                elev = round(elevs_sorted[ring // 2])
                spd = round(speeds_sorted[ring // 2])

                # rospy.loginfo("{}: {}UTC: {:.5f}N {:.5f}E @ {:.1f}M, {:.1f}km/h, {}".format(
                #     rospy.get_caller_id(),
                #     gps_time,
                #     latitude,
                #     longitude,
                #     elev,
                #     spd,
                #     fix_quality
                # ))

                gps_pub.publish(GPSMsg(latitude, longitude, elev, spd, fix_quality))

            else:
                if fixed:
                    rospy.loginfo("{}: Fix Lost!".format(rospy.get_caller_id()))
                    fixed = False

            rate.sleep()

        except Exception as e:
            pass
            # rospy.logerr("{}: {}".format(rospy.get_caller_id(), e))


if __name__ == '__main__':

    try:

        serial_port = '/dev/' + rospy.get_param("/serial/gps_vk")
        ser = serial.Serial(serial_port, 9600, timeout=1)

        main(ser)

    except rospy.ROSInterruptException as error:
        pass
