#!/usr/bin/env python3

"""
GPS Interface

Interface which reads raw data from the VK-162 GPS module and 
outputs the position (Latitude and Longitude) of the vehicle.

"""

import time
import serial

def readString():
    while 1:
        while ser.read().decode("utf-8") != '$':  # Wait for the begging of the string
            pass  # Do nothing
        line = ser.readline().decode("utf-8")  # Read the entire string
        return line


def getTime(string, format, returnFormat):
    return time.strftime(returnFormat,
                         time.strptime(string, format))  # Convert date and time to a nice printable format


def getLatLng(latString, lngString):
    lat = latString[:2].lstrip('0') + "." + "%.7s" % str(float(latString[2:]) * 1.0 / 60.0).lstrip("0.")
    lng = lngString[:3].lstrip('0') + "." + "%.7s" % str(float(lngString[3:]) * 1.0 / 60.0).lstrip("0.")
    return lat, lng


def printRMC(lines):
    print("========================================RMC========================================")
    # print(lines, '\n')
    print("Fix taken at:", getTime(lines[1] + lines[9], "%H%M%S.%f%d%m%y", "%a %b %d %H:%M:%S %Y"), "UTC")
    print("Status (A=OK,V=KO):", lines[2])
    latlng = getLatLng(lines[3], lines[5])
    print("Lat,Long: ", latlng[0], lines[4], ", ", latlng[1], lines[6], sep='')
    print("Speed (knots):", lines[7])
    print("Track angle (deg):", lines[8])
    print("Magnetic variation: ", lines[10], end='')
    if len(
            lines) == 13:  # The returned string will be either 12 or 13 - it will return 13 if NMEA standard used is above 2.3
        print(lines[11])
        print("Mode (A=Autonomous, D=Differential, E=Estimated, N=Data not valid):", lines[12].partition("*")[0])
    else:
        print(lines[11].partition("*")[0])

    return


def printGGA(lines):
    print("========================================GGA========================================")
    # print(lines, '\n')
    print("Fix taken at:", getTime(lines[1], "%H%M%S.%f", "%H:%M:%S"), "UTC")
    latlng = getLatLng(lines[2], lines[4])
    print("Lat,Long: ", latlng[0], lines[3], ", ", latlng[1], lines[5], sep='')
    print("Fix quality (0 = invalid, 1 = fix, 2..8):", lines[6])
    print("Satellites:", lines[7].lstrip("0"))
    print("Horizontal dilution:", lines[8])
    print("Altitude: ", lines[9], lines[10], sep="")
    print("Height of geoid: ", lines[11], lines[12], sep="")
    print("Time in seconds since last DGPS update:", lines[13])
    print("DGPS station ID number:", lines[14].partition("*")[0])
    return


def printGSA(lines):
    print("========================================GSA========================================")
    # print(lines, '\n')

    print("Selection of 2D or 3D fix (A=Auto,M=Manual):", lines[1])
    print("3D fix (1=No fix,2=2D fix, 3=3D fix):", lines[2])
    print("PRNs of satellites used for fix:", end='')
    for i in range(0, 12):
        prn = lines[3 + i].lstrip("0")
        if prn:
            print(" ", prn, end='')
    print("\nPDOP", lines[15])
    print("HDOP", lines[16])
    print("VDOP", lines[17].partition("*")[0])
    return


def printGSV(lines):
    if lines[2] == '1':  # First sentence
        print("========================================GSV========================================")
    else:
        print("===================================================================================")
    # print(lines, '\n')

    print("Number of sentences:", lines[1])
    print("Sentence:", lines[2])
    print("Satellites in view:", lines[3].lstrip("0"))
    for i in range(0, int(len(lines) / 4) - 1):
        print("Satellite PRN:", lines[4 + i * 4].lstrip("0"))
        print("Elevation (deg):", lines[5 + i * 4].lstrip("0"))
        print("Azimuth (deg):", lines[6 + i * 4].lstrip("0"))
        print("SNR (higher is better):", lines[7 + i * 4].partition("*")[0])
    return


def printGLL(lines):
    print("========================================GLL========================================")
    # print(lines, '\n')

    latlng = getLatLng(lines[1], lines[3])
    print("Lat,Long: ", latlng[0], lines[2], ", ", latlng[1], lines[4], sep='')
    print("Fix taken at:", getTime(lines[5], "%H%M%S.%f", "%H:%M:%S"), "UTC")
    print("Status (A=OK,V=KO):", lines[6])
    if lines[7].partition("*")[0]:  # Extra field since NMEA standard 2.3
        print("Mode (A=Autonomous, D=Differential, E=Estimated, N=Data not valid):", lines[7].partition("*")[0])
    return


def printVTG(lines):
    print("========================================VTG========================================")
    # print(lines, '\n')

    print("True Track made good (deg):", lines[1], lines[2])
    print("Magnetic track made good (deg):", lines[3], lines[4])
    print("Ground speed (knots):", lines[5], lines[6])
    print("Ground speed (km/h):", lines[7], lines[8].partition("*")[0])
    if lines[9].partition("*")[0]:  # Extra field since NMEA standard 2.3
        print("Mode (A=Autonomous, D=Differential, E=Estimated, N=Data not valid):", lines[9].partition("*")[0])
    return


def checksum(line):
    checkString = line.partition("*")
    checksum = 0
    for c in checkString[0]:
        checksum ^= ord(c)

    try:  # Just to make sure
        inputChecksum = int(checkString[2].rstrip(), 16);
    except:
        print("Error in string")
        return False

    if checksum == inputChecksum:
        return True
    else:
        print("=====================================================================================")
        print("===================================Checksum error!===================================")
        print("=====================================================================================")
        print(hex(checksum), "!=", hex(inputChecksum))
        return False

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Open Serial port
    try:
        while True:
            line = readString()
            lines = line.split(",")
            if checksum(line):
                if lines[0] == "GPRMC":
                    printRMC(lines)
                    pass
                elif lines[0] == "GPGGA":
                    printGGA(lines)
                    pass
                elif lines[0] == "GPGSA":
                    # printGSA(lines)
                    pass
                elif lines[0] == "GPGSV":
                    # printGSV(lines)
                    pass
                elif lines[0] == "GPGLL":
                    printGLL(lines)
                    pass
                elif lines[0] == "GPVTG":
                    printVTG(lines)
                    pass
                else:
                    print("\n\nUnknown type:", lines[0], "\n\n")
    except KeyboardInterrupt:
        print('Exiting Script')
