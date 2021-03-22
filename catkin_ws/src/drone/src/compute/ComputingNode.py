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
from drone.msg import ComputeMsg
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
PIXEL_SIZE = 0.0  # cm

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
CSV_HEADER = ['time', 'stamp',
              'people', 'warning', 'density',
              'latitude', 'longitude', 'sat',
              'altitude[m]', 'height[m]', 'temp[°C]', 'angle[deg]', 'area[m2]', 'battery[%]']
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
DETECTION_RING = 10
PREV_OVERLAY = None
PREV_DETECTIONS = []
OVERLAY_INDEX = 0
BEST_OVERLAY = None
BEST_DETECTIONS = []

# social distancing threshold
DISTANCING_THRESHOLD = 150

# the ROS publisher that sends crowd info
compute_pub = None


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
    LAT_LNG = (round(data.lat, 4), round(data.lng, 4))
    SAT = data.fix


def attitude_callback(data):
    global BATTERY
    BATTERY = data.percentage


def resolveHeight():
    global H
    # if SONAR_MIN <= SONAR <= SONAR_MAX:
    #     H = SONAR / 100  # convert cm to m
    # elif REL_ALT > 0:
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
        return 0.001

    AREA_DIST_VERT = coeff * H

    computePixelSize(frame_height=1232)

    return round(AREA_DIST_VERT * AREA_DIST_HORIZ, 2)


def computePixelSize(frame_height):
    # computes the relative size of a pixel
    # projected onto the ground plane
    # by knowing the projected area and the captured frames' respective width and height
    # this enables computing the relative distance between objects in physical units (m or cm)
    global PIXEL_SIZE

    PIXEL_SIZE = (AREA_DIST_VERT / frame_height) * 100


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

            density = round((people_count / CAM_AREA) * 10, 2)

            # check social distancing
            overlay, neck_breathers = check_social_distancing(detections, overlay)

            rospy.loginfo(
                '{}: {} people, {}/10m2 density, {} too close'.format(
                    rospy.get_caller_id(), people_count, density, neck_breathers))

            compute_pub.publish(ComputeMsg(people_count, neck_breathers, density, CAM_AREA))

            # save data to the csv file
            if CSV_PATH is not None:
                with open(CSV_PATH, 'a', newline='') as csv_file:
                    writer = csv.writer(csv_file)
                    row = [time_of_day, timestamp,
                           people_count, neck_breathers, density,
                           LAT_LNG[0], LAT_LNG[1], SAT,
                           ABS_ALT, H, AIR_TEMP, rospy.get_param('/physical/camera_angle'), CAM_AREA, BATTERY]
                    writer.writerow(row)

            # save the overlaid image
            cv2.imwrite('/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/random/{}_{}.jpg'.format(
                timestamp, people_count),
                overlay)

    else:
        rospy.loginfo('{}: No people detected!'.format(rospy.get_caller_id()))


def video_mock_test():
    global PREV_DETECTIONS, PREV_OVERLAY, BEST_DETECTIONS, BEST_OVERLAY, OVERLAY_INDEX, H

    video = cv2.VideoCapture('/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/videos/fountain.mp4')

    while video.isOpened():
        ret, frame = video.read()

        if not ret:
            break

        resized = cv2.resize(frame, (1080, 1920), interpolation=cv2.INTER_NEAREST)

        overlay, detections = detector.run_inference(resized, 3, 5)

        current_people = len(detections)  # total number of people in the image

        # there is at least one person in the frame
        if current_people > 0:
            OVERLAY_INDEX += 1
            # find the image with the maximum number of people
            if current_people > len(BEST_DETECTIONS):
                BEST_DETECTIONS = detections
                BEST_OVERLAY = overlay

            PREV_DETECTIONS = detections
            PREV_OVERLAY = overlay

            # the maximum from the previous detections has been established
            if OVERLAY_INDEX == DETECTION_RING:
                OVERLAY_INDEX = 0
                people_count = len(BEST_DETECTIONS)
                overlay = BEST_OVERLAY
                detections = BEST_DETECTIONS
                time_of_day = str(datetime.now().strftime('%H:%M:%S'))
                timestamp = round(time.time())

                H = 10.0
                area = computeVisibleCamArea(45)

                density = round((people_count / area) * 10, 2)

                # check social distancing
                overlay, neck_breathers = check_social_distancing(detections, overlay)

                rospy.loginfo(
                    '{}: {} people, {}/10m2 density, {} too close'.format(
                        rospy.get_caller_id(), people_count, density, neck_breathers))

                # save data to the csv file
                if CSV_PATH is not None:
                    with open(CSV_PATH, 'a', newline='') as csv_file:
                        writer = csv.writer(csv_file)
                        row = [time_of_day, timestamp,
                               people_count, neck_breathers, density,
                               LAT_LNG[0], LAT_LNG[1], SAT,
                               ABS_ALT, 10, AIR_TEMP, 45, area, BATTERY]
                        writer.writerow(row)

                # save the overlaid image
                cv2.imwrite('/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/random/{}_{}.jpg'.format(
                    timestamp, people_count),
                    overlay)

    else:
        rospy.loginfo('{}: No people detected!'.format(rospy.get_caller_id()))

    video.release()
    rospy.loginfo('{}: Video processing done!'.format(rospy.get_caller_id()))


def projected_distance(detection_box_1, detection_box_2):
    b1X = detection_box_1.Center[0]
    b1Y = detection_box_1.Center[1]

    b2X = detection_box_2.Center[0]
    b2Y = detection_box_2.Center[1]

    pixel_distance = math.sqrt((b2X - b1X) ** 2 + (b2Y - b1Y) ** 2)
    physical_distance = pixel_distance * PIXEL_SIZE  # cm

    return physical_distance


def check_social_distancing(detections, overlay):
    highlighted = []
    # estimates distances between people from the air
    for idx1 in range(len(detections) - 1):
        for idx2 in range(idx1 + 1, len(detections)):
            physical_distance = projected_distance(detections[idx1], detections[idx2])
            if physical_distance < DISTANCING_THRESHOLD:
                # highlight people who are too close to each other
                cv2.rectangle(overlay,
                              (int(detections[idx1].Left), int(detections[idx1].Top)),
                              (int(detections[idx1].Right), int(detections[idx1].Bottom)),
                              color=(0, 0, 255),
                              thickness=2)

                cv2.rectangle(overlay,
                              (int(detections[idx2].Left), int(detections[idx2].Top)),
                              (int(detections[idx2].Right), int(detections[idx2].Bottom)),
                              color=(0, 0, 255),
                              thickness=2)

                if idx1 not in highlighted:
                    highlighted.append(idx1)
                if idx2 not in highlighted:
                    highlighted.append(idx2)

    neck_breathers = len(highlighted)

    return overlay, neck_breathers


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
    global DETECTION_STARTED, CAM_AREA, CSV_FILENAME, CSV_PATH, compute_pub

    rospy.init_node('ComputingNode')

    rospy.Subscriber('SonarReading', Float32, sonar_callback)
    rospy.Subscriber('Altitude', AltitudeMsg, alt_callback)
    rospy.Subscriber('FrameSaved', Bool, frame_saved_callback)
    rospy.Subscriber('FrameSaved', Bool, frame_saved_callback)
    rospy.Subscriber('GPS', GPSMsg, gps_callback)
    rospy.Subscriber('CraftAttitude', AttitudeMsg, attitude_callback)

    compute_pub = rospy.Publisher('Compute', ComputeMsg, queue_size=1)

    # load a pre-trained network, using TensorRT
    detector.load_net('ssd-mobilenet-v2', threshold=0.35)

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

        # rospy.loginfo('{}: {}cm/pixel'.format(rospy.get_caller_id(), round(PIXEL_SIZE, 1)))
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
