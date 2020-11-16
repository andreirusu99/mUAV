#!/usr/bin/env python3
import rospy
from drone.msg import ControlAxes as ControlAxesMsg
from drone.msg import Attitude as AttitudeMsg

from yamspy import MSPy
import time
import threading
import os

FCinfo = ['MSP_ANALOG', 'MSP_ATTITUDE']

CMDS = {
    'roll':     1500,
    'pitch':    1500,
    'throttle': 1000,
    'yaw':      1500,
    'aux1':     1000,  # arming
    'aux2':     1000  # camera servo
}

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

INFO_PERIOD = 1

STICK_MIN = 1200
STICK_MAX = 1800
THROTTLE_MAX = 1700

ROLL_TRIM = 15
PITCH_TRIM = -17
YAW_TRIM = 0

def clamp(n, low, high):
    return max(min(high, n), low)


def getFCinfo(drone):
    power = percentage = 0.0
    roll = pitch = yaw = 0.0
    for next_msg in FCinfo:

        if drone.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
            dataHandler = drone.receive_msg()
            drone.process_recv_data(dataHandler)

        if next_msg == "MSP_ANALOG":
            voltage = drone.ANALOG['voltage']
            amperage = drone.ANALOG['amperage']
            power = voltage * amperage
            percentage = (voltage - 9.9) / 2.73 * 100
            percentage = min(max(percentage, 0), 100)

        if next_msg == "MSP_ATTITUDE":
            roll = drone.SENSOR_DATA['kinematics'][0]
            pitch = drone.SENSOR_DATA['kinematics'][1]
            yaw = drone.SENSOR_DATA['kinematics'][2]

    return power, percentage, roll, pitch, yaw


def control_callback(data):
    global CMDS
    CMDS['roll'] = clamp(data.axis[0], STICK_MIN, STICK_MAX) + ROLL_TRIM
    CMDS['pitch'] = clamp(data.axis[1], STICK_MIN, STICK_MAX) + PITCH_TRIM
    CMDS['throttle'] = clamp(data.axis[2], 1000, THROTTLE_MAX)
    CMDS['yaw'] = clamp(data.axis[3], STICK_MIN, STICK_MAX) + YAW_TRIM

    applyDeadZone()


def applyDeadZone():
    global CMDS
    # deadzone configuration
    dead_zone_ratio = 0.1
    input_range = 1000.0
    dead_zone = input_range * dead_zone_ratio
    roll, pitch, throttle, yaw = CMDS['roll'], CMDS['pitch'], CMDS['throttle'], CMDS['yaw']

    if abs(roll - 1500) < dead_zone:
        roll = 1500
    if abs(pitch - 1500) < dead_zone:
        pitch = 1500
    if abs(yaw - 1500) < dead_zone * 1.5:
        yaw = 1500
    if abs(throttle - 1000) < 50:
        throttle = 1000

    CMDS['roll'], CMDS['pitch'], CMDS['throttle'], CMDS['yaw'] = roll, pitch, throttle, yaw


def main(drone):

    rospy.init_node('Dispatcher')

    # subscribe to get control axes from Interceptor
    rospy.Subscriber('Control', ControlAxesMsg, control_callback)

    # publish Attitude and Power info
    attitude_pub = rospy.Publisher('CraftAttitude', AttitudeMsg, queue_size=1)

    drone.is_ser_open = not drone.connect(trials=drone.ser_trials)
    if drone.is_ser_open:
        rospy.loginfo("{}: Connected to FC on {}".format(
            rospy.get_caller_id(), serial_port))
    else:
        rospy.logerr("{}: Error opening serial port.".format(
            rospy.get_caller_id()))
        os._exit(1)

    last_info = time.time()
    while not rospy.is_shutdown():

        armed = rospy.get_param("/run/armed")

        # get board info
        power, percentage, roll, pitch, yaw = getFCinfo(drone)

        # arming and disarming
        CMDS['aux1'] = 2000 if armed else 1000

        # compensate camera angle pitch
        camera_angle = clamp(rospy.get_param("/physical/camera_angle") + pitch, 0, 90)

        # setting the camera angle
        CMDS['aux2'] = round(1000 + (11.111 * camera_angle))

        # send the channels to the board
        if(drone.send_RAW_RC([CMDS[i] for i in CMDS_ORDER])):
            dataHandler = drone.receive_msg()
            drone.process_recv_data(dataHandler)

        attitude_pub.publish(AttitudeMsg(roll, pitch, yaw, percentage, power))

        rospy.set_param("/physical/roll", roll)
        rospy.set_param("/physical/pitch", pitch)
        rospy.set_param("/physical/yaw", yaw)

        rospy.set_param("/run/power", round(power))
        rospy.set_param("/run/battery", round(percentage))


        if time.time() - last_info > INFO_PERIOD:
            rospy.loginfo("{}: {:.0f}% left @ {:.0f}W, (R{:.2f}, P{:.2f}, Y{:.2f}) -> {}".format(
                rospy.get_caller_id(), percentage, power, roll, pitch, yaw, CMDS['throttle']))
            last_info = time.time()


if __name__ == '__main__':

    try:

        serial_port = "/dev/" + rospy.get_param("/serial/flight_controller")
        drone = MSPy(device=serial_port, loglevel="INFO", baudrate=115200)

        main(drone)

    except rospy.ROSInterruptException as error:
        drone.conn.close()
