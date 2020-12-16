#!/usr/bin/env python3

import cv2
import math
import time
import numpy as np
import threading

import rospy
from drone.msg import Altitude as AltitudeMsg
from std_msgs.msg import Float32
from std_msgs.msg import Bool

from src.compute import Detector as detector

# the current camera frame
FRAME = None

# the above frame, with overlays, in numpy format
FRAME_OVERLAY_NP = None

# the ground height from the sonar
SONAR = 0.0  # cm

# the distance to the ground
H = 0.0  # m

# the side lengths of the projection (rectangle) of the camera frustum on the ground
AREA_DIST_VERT = 0.0  # m
AREA_DIST_HORIZ = 0.0  # m

# the altitudes from the barometer
REL_ALT = 0.0  # m
ABS_ALT = 0.0  # m

# PiCam intrinsic parameters
CAM_V_FOV = 48.8  # deg

# cm from which sonar reading is ignored
SONAR_MAX = 300  # cm

# cm until which the craft is considered to be stationary (landed)
SONAR_MIN = 2  # cm

# the side length of one pixel, projected onto the ground plane
PIXEL_SIZE_V = 0.0  # cm
PIXEL_SIZE_H = 0.0  # cm

# number of horizontal and vertical regions
# in which to tile the input image
# for better accuracy in detection
WIDTH_TILES = 4
HEIGHT_TILES = 3
DETECTION_STARTED = False

FRAME_READY = False


def sonar_callback(data):
    global SONAR
    SONAR = data.data
    resolveHeight()


def alt_callback(data):
    global REL_ALT, ABS_ALT
    REL_ALT = data.relative
    ABS_ALT = data.absolute
    resolveHeight()


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
        return

    AREA_DIST_VERT = coeff * H

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


def detectionThread():
    global FRAME_OVERLAY_NP, FRAME_READY
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
            frame = cv2.filter2D(frame, -1, 0.75 * kernel)

            # run inference on the image
            start_inference = time.time()
            FRAME_OVERLAY_NP, detections = detector.run_inference(frame, WIDTH_TILES, HEIGHT_TILES)
            end_inference = time.time()

            rospy.loginfo("{}: Detection {:.0f}ms, {} people"
                          .format(rospy.get_caller_id(), (end_inference - start_inference) * 1000, len(detections)))

            # save the overlaid image
            cv2.imwrite(
                "/home/andrei/Desktop/mUAV/catkin_ws/src/drone/data/out/random/"
                + str(time.time()) + '_' + str(len(detections)) + '.jpg',
                FRAME_OVERLAY_NP)

            # flag the end of frame processing
            FRAME_READY = False

            # request another frame
            frame_pub.publish(Bool(True))


def main():
    global DETECTION_STARTED
    rospy.init_node('ComputingNode')

    rospy.Subscriber('SonarReading', Float32, sonar_callback)
    rospy.Subscriber('Altitude', AltitudeMsg, alt_callback)
    rospy.Subscriber('FrameSaved', Bool, frame_saved_callback)

    area_pub = rospy.Publisher('Area', Float32, queue_size=1)

    # load a pre-trained network, using TensorRT
    detector.load_net("ssd-mobilenet-v2", threshold=0.4)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cam_angle = rospy.get_param("/physical/camera_angle")
        DETECTION_STARTED = rospy.get_param("/run/detection_started")

        # computing the visible area
        cam_area = computeVisibleCamArea(cam_angle)

        # computing the pixel size
        computePixelSize(1280, 720)

        # publishing
        area_pub.publish(Float32(cam_area))

        # rospy.loginfo("{}: {}H, {}V".format(rospy.get_caller_id(), round(PIXEL_SIZE_H, 1), round(PIXEL_SIZE_V, 1)))
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
