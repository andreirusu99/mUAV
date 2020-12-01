#!/usr/bin/env python3

import cv2
import math
import time
import numpy as np
import threading

import jetson.inference
import jetson.utils

import rospy
from drone.msg import Altitude as AltitudeMsg
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from src.sensors import Camera as cam

# the current camera frame
FRAME = None

# the above frame with detection overlays, in cudaImage format
CUDA_FRAME = None

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

bridge = CvBridge()


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


def main():
    global FRAME, CUDA_FRAME, FRAME_OVERLAY_NP
    rospy.init_node('ComputingNode')

    rospy.Subscriber('SonarReading', Float32, sonar_callback)
    rospy.Subscriber('Altitude', AltitudeMsg, alt_callback)

    area_pub = rospy.Publisher('Area', Float32, queue_size=1)
    img_pub = rospy.Publisher('Camera', Image)

    net = jetson.inference.detectNet("ssd-inception-v2", threshold=0.3)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        cam_angle = rospy.get_param("/physical/camera_angle")
        FRAME = cam.FRAME
        CUDA_FRAME = cam.CUDA_FRAME

        # computing the visible area
        cam_area = computeVisibleCamArea(cam_angle)

        # computing the pixel size
        computePixelSize(FRAME.shape[1], FRAME.shape[0])

        # computing detections on cudaImage object
        detections = net.Detect(CUDA_FRAME, FRAME.shape[1], FRAME.shape[0])
        # rospy.loginfo("{}: {} detections, max {:.1f}FPS".format(
        #     rospy.get_caller_id(), len(detections), net.GetNetworkFPS()))

        # after detection, convert CUDA_FRAME to np array in BGR mode
        FRAME_OVERLAY_NP = jetson.utils.cudaToNumpy(CUDA_FRAME, FRAME.shape[1], FRAME.shape[0], FRAME.shape[2])
        FRAME_OVERLAY_NP = cv2.cvtColor(FRAME_OVERLAY_NP, cv2.COLOR_RGB2BGR)
        FRAME_OVERLAY_NP = np.asarray(FRAME_OVERLAY_NP)

        # publishing
        area_pub.publish(Float32(cam_area))

        try:
            img_pub.publish(bridge.cv2_to_imgmsg(FRAME, encoding="bgr8"))
        except CvBridgeError as e:
            print(e)

        # rospy.loginfo("{}: {}H, {}V".format(rospy.get_caller_id(), round(PIXEL_SIZE_H, 1), round(PIXEL_SIZE_V, 1)))
        rate.sleep()


if __name__ == '__main__':
    try:
        # start the camera thread
        camera_thread = threading.Thread(target=lambda: cam.captureFrames())
        camera_thread.daemon = True
        camera_thread.start()

        main()

        camera_thread.join(1)

    except rospy.ROSInterruptException as error:
        pass
