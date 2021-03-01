#!/usr/bin/env python3

import cv2
import math
import time
import numpy as np
import threading
from datetime import datetime
import csv
import os

import rospy
from drone.msg import Altitude as AltitudeMsg
from drone.msg import Attitude as AttitudeMsg
from drone.msg import GPSinfo as GPSMsg
from std_msgs.msg import Float32
from std_msgs.msg import Bool

from src.compute import Detector as detector

# the current camera frame
FRAME = None

# the above frame, with overlays, in numpy format
FRAME_OVERLAY_NP = None

# the ground height from the sonar
SONAR = 0.0  # cm

# the altitudes from the barometer
REL_ALT = 0.0  # m
ABS_ALT = 0.0  # m

# the distance to the ground
H = 0.0  # m

# the side lengths of the projection (rectangle) of the camera frustum on the ground
AREA_DIST_VERT = 0.0  # m
AREA_DIST_HORIZ = 0.0  # m

# PiCam intrinsic parameters
CAM_V_FOV = 48.8  # deg

# cm from which sonar reading is ignored
SONAR_MAX = 300  # cm

# cm until which the craft is considered to be stationary (landed)
SONAR_MIN = 2  # cm

# the side length of one pixel, projected onto the ground plane
PIXEL_SIZE_V = 0.0  # cm
PIXEL_SIZE_H = 0.0  # cm

# ground area visible by the camera
CAM_AREA = 0.0  # m2

# GPS location of craft and satellites
LAT_LNG = (0.0, 0.0)
SAT = 0

# battery level
BATTERY = 0

# outside air temperature
AIR_TEMP = 0

# CSV file header
CSV_HEADER = ['time', 'timestamp',
              'people_count', 'density[ppl/m2]',
              'latitude', 'longitude', 'sat',
              'altitude[m]', 'height[m]', 'air_temp[°C]', 'angle[deg]', 'area[m2]', 'battery[%]']
CSV_FILENAME = ''
CSV_LOCATION = '/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/csv/'
CSV_PATH = ''

# number of horizontal and vertical regions
# in which to tile the input image
# for better accuracy in detection
WIDTH_TILES = 4
HEIGHT_TILES = 3
DETECTION_STARTED = False

FRAME_READY = False

# the number of last frames to consider for processing
DETECTION_RING = 7
PREV_OVERLAY = None
PREV_DETECTIONS = []
OVERLAY_INDEX = 0
BEST_OVERLAY = None
BEST_DETECTIONS = []


def sonar_callback(data):
    global SONAR
    SONAR = data.data
    resolveHeight()


def alt_callback(data):
    global REL_ALT, ABS_ALT, AIR_TEMP
    REL_ALT = round(data.relative, 1)
    ABS_ALT = round(data.absolute, 1)
    AIR_TEMP = round(data.temp, 1)
    resolveHeight()


def gps_callback(data):
    global LAT_LNG, SAT
    LAT_LNG = (round(data.lat, 4), round(data.lng), 4)
    SAT = data.fix


def attitude_callback(data):
    global BATTERY
    BATTERY = data.percentage


def resolveHeight():
    global H
    if SONAR_MIN <= SONAR <= SONAR_MAX:
        H = SONAR / 100  # convert cm to m
    elif REL_ALT > 0:
        H = REL_ALT


def computeVisibleCamArea(cam_angle):
    # computes the total area of flat ground visible by the camera
    # with respect to the craft's distance from the ground H
    # and knowing the camera's tilt angle
    global AREA_DIST_VERT, AREA_DIST_HORIZ

    AREA_DIST_HORIZ = 1.2 * H  # fixed, since camera cannot move side-to-side
    # 1.2 coefficient calculated from the cosine of the camera's horizontal FOV

    # computing the vertical length area coefficient
    # = the multiplier of the vertical length of the rectangle that describes the visible area of the camera
    # -> depends on the angle that the camera is angled at the ground
    theta = cam_angle - (CAM_V_FOV / 2)
    alpha = 90.0 - theta - CAM_V_FOV
    coeff = 1.0 / math.tan(math.radians(alpha)) - math.sin(math.radians(theta))

    if coeff < 0:
        return 1

    AREA_DIST_VERT = coeff * H

    computePixelSize(1632, 1232)

    return round(AREA_DIST_VERT * AREA_DIST_HORIZ, 2)


def computePixelSize(frame_width, frame_height):
    # computes the relative size of a pixel
    # projected onto the ground plane
    # by knowing the projected area and the capture frames' respective width and height
    # this enables computing the relative distance between objects in measurable units (m or cm)
    global PIXEL_SIZE_V, PIXEL_SIZE_H

    PIXEL_SIZE_H = (AREA_DIST_HORIZ / frame_width) * 100
    PIXEL_SIZE_V = (AREA_DIST_VERT / frame_height) * 100


def frame_saved_callback(data):
    global FRAME_READY, FRAME
    FRAME = cv2.imread('/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/live/frame.jpg')
    FRAME_READY = True


def process_data(current_overlay, current_detections):
    global PREV_DETECTIONS, PREV_OVERLAY, BEST_DETECTIONS, BEST_OVERLAY, OVERLAY_INDEX
    # Example Detection class
    # ClassID: 1
    # -- Confidence: 0.409221
    # -- Left: 299.436
    # -- Top: 349.765
    # -- Right: 324.057
    # -- Bottom: 403.88
    # -- Width: 24.6211
    # -- Height: 54.115
    # -- Area: 1332.37
    # -- Center: (311.746, 376.823)

    current_people = len(current_detections)  # total number of people in the image
    # there is at least one person in the frame
    if current_people > 0:
        OVERLAY_INDEX += 1
        # find the image with the maximum number of people
        if current_people > len(BEST_DETECTIONS):
            BEST_DETECTIONS = current_detections
            BEST_OVERLAY = current_overlay

        PREV_DETECTIONS = current_detections
        PREV_OVERLAY = current_overlay

        # the maximum from the previous detections has been established
        if OVERLAY_INDEX == DETECTION_RING:
            OVERLAY_INDEX = 0
            people_count = len(BEST_DETECTIONS)
            overlay = BEST_OVERLAY
            detections = BEST_DETECTIONS
            time_of_day = str(datetime.now().strftime('%H:%M:%S'))
            timestamp = round(time.time())

            density = round(people_count / CAM_AREA, 1)

            rospy.loginfo('{}: {} people, {}/m2 density'.format(rospy.get_caller_id(), people_count, density))

            # save data to the csv file
            if CSV_PATH is not None:
                with open(CSV_PATH, 'a', newline='') as csv_file:
                    writer = csv.writer(csv_file)
                    row = [time_of_day, timestamp,
                           people_count, density,
                           LAT_LNG[0], LAT_LNG[1], SAT,
                           ABS_ALT, H, AIR_TEMP, rospy.get_param('/physical/camera_angle'), CAM_AREA, BATTERY]
                    writer.writerow(row)

            # save the overlaid image
            # TODO: save to CSV file + GPS location
            cv2.imwrite('/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/random/{}_{}.jpg'.format(
                timestamp, people_count),
                overlay)

    else:
        rospy.loginfo('{}: No people detected!'.format(rospy.get_caller_id()))


def detectionThread():
    global FRAME_OVERLAY_NP, FRAME_READY, OVERLAY_INDEX
    frame_pub = rospy.Publisher('FrameRequested', Bool, queue_size=1)

    while True:

        if not DETECTION_STARTED or not FRAME_READY:
            continue

        if FRAME is not None:
            frame = FRAME.copy()
            # sharpening the input image
            kernel = np.array([[0, -1, 0],
                               [-1, 5, -1],
                               [0, -1, 0]])
            # frame = cv2.filter2D(frame, -1, 0.75 * kernel)

            # run inference on the camera image continuously
            # start_inference = time.time()
            FRAME_OVERLAY_NP, detections = detector.run_inference(frame, WIDTH_TILES, HEIGHT_TILES)
            # end_inference = time.time()

            # rospy.loginfo('{}: Detected {} people in {:.0f}ms, index {}'
            #               .format(rospy.get_caller_id(),
            #                       len(detections),
            #                       (end_inference - start_inference) * 1000,
            #                       OVERLAY_INDEX))

            # process the frames and detections to produce useful information
            process_data(FRAME_OVERLAY_NP.copy(), detections)

            # flag the end of frame processing
            FRAME_READY = False

            # request another frame
            frame_pub.publish(Bool(True))


def main():
    global DETECTION_STARTED, CAM_AREA, CSV_FILENAME, CSV_PATH
    rospy.init_node('ComputingNode')

    rospy.Subscriber('SonarReading', Float32, sonar_callback)
    rospy.Subscriber('Altitude', AltitudeMsg, alt_callback)
    rospy.Subscriber('FrameSaved', Bool, frame_saved_callback)
    rospy.Subscriber('FrameSaved', Bool, frame_saved_callback)
    rospy.Subscriber('GPS', GPSMsg, gps_callback)
    rospy.Subscriber('CraftAttitude', AttitudeMsg, attitude_callback)

    # load a pre-trained network, using TensorRT
    detector.load_net('ssd-mobilenet-v2', threshold=0.4)

    # create the name of the CSV file (date of today)
    CSV_FILENAME = str(datetime.now().strftime('%d%h%y')) + '.csv'
    CSV_PATH = CSV_LOCATION + CSV_FILENAME

    # if a CSV file does not exist for this date, create it
    for _, _, files in os.walk(CSV_LOCATION):
        if CSV_FILENAME not in files:
            with open(CSV_PATH, 'w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(CSV_HEADER)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cam_angle = rospy.get_param('/physical/camera_angle')
        DETECTION_STARTED = rospy.get_param('/run/detection_started')

        # computing the visible area
        CAM_AREA = computeVisibleCamArea(cam_angle)

        # rospy.loginfo('{}: {}H, {}V'.format(rospy.get_caller_id(), round(PIXEL_SIZE_H, 1), round(PIXEL_SIZE_V, 1)))
        rate.sleep()


if __name__ == '__main__':
    try:

        # start the object detection thread
        detector_thread = threading.Thread(target=detectionThread)
        detector_thread.daemon = True
        detector_thread.start()

        # main thread
        main()

    except rospy.ROSInterruptException as error:
        pass
