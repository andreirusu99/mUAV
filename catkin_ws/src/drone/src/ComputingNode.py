#!/usr/bin/env python3
import math
import time

import rospy
from drone.msg import Altitude as AltitudeMsg
from std_msgs.msg import Float32

# the ground height from the sonar
SONAR = 0.0

# the distance to the ground
H = 0.0

# the width and length of the projection (reactangle) of the camera frustum on the ground
AREA_VERT = 0.0
AREA_HORIZ = 0.0

# the altitudes from the barometer
REL_ALT = 0.0
ABS_ALT = 0.0

# PiCam intrinsic parameters
CAM_V_FOV = 48.8


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
    if 5 <= SONAR <= 300:
        H = SONAR / 100  # convert cm to m
    elif REL_ALT > 0:
        H = REL_ALT


def computeVisibleCamArea(cam_angle):
    global AREA_VERT, AREA_HORIZ

    AREA_HORIZ = 1.2 * H  # fixed, since camera cannot move side-to-side

    theta = cam_angle - (CAM_V_FOV / 2)
    alpha = 90.0 - (CAM_V_FOV / 2) - cam_angle
    coeff = 1.0 / math.tan(math.radians(alpha)) - math.sin(math.radians(theta))

    if coeff < 0:
        return

    AREA_VERT = H * coeff

    AREA_VERT = round(AREA_VERT, 2)
    AREA_HORIZ = round(AREA_HORIZ, 2)

    # rospy.loginfo(
    #     "{}: {:.2f}m X {:.2f}m, H={:.2f}m, coeff={:.2f}".format(rospy.get_caller_id(), AREA_HORIZ, AREA_VERT, H, coeff))


def main():
    rospy.init_node('ComputingNode')

    rospy.Subscriber('SonarReading', Float32, sonar_callback)
    rospy.Subscriber('Altitude', AltitudeMsg, alt_callback)

    last_print = 0
    while not rospy.is_shutdown():
        cam_angle = rospy.get_param("/physical/camera_angle")

        if time.time() - last_print > 0.5:
            computeVisibleCamArea(cam_angle)
            rospy.loginfo("{}: {}m^2".format(rospy.get_caller_id(), round(AREA_HORIZ * AREA_VERT, 2)))
            last_print = time.time()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as error:
        pass
