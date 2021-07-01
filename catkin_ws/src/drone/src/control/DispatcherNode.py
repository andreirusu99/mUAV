#!/usr/bin/env python3
import time

import rospy
from drone.msg import Altitude as AltitudeMsg
from drone.msg import Attitude as AttitudeMsg
from drone.msg import ControlAxes as ControlAxesMsg
from drone.msg import RunInfo as RunInfoMsg
from std_msgs.msg import Bool
from yamspy import MSPy

from src.sensors import BMP_Interface as bmp

FCinfo = ['MSP_ANALOG', 'MSP_ATTITUDE']

CMDS = {
    'roll': 1500,
    'pitch': 1500,
    'throttle': 1000,
    'yaw': 1500,
    'aux1': 1000,  # arming
    'aux2': 1000  # camera servo
}

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

INFO_PERIOD = 1  # seconds
SEND_PERIOD = 0.1  # seconds

# controls stick limits
STICK_MIN = 1000
STICK_MAX = 2000
THROTTLE_MAX = 1700

# controls stick dead zone and scaling
DEAD_ZONE = 0.05
STICK_SCALE = 0.5


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
            power = round(voltage * amperage)
            percentage = (voltage - 9.9) / 2.73 * 100
            percentage = round(clamp(percentage, 0, 100))

        if next_msg == "MSP_ATTITUDE":
            roll = round(drone.SENSOR_DATA['kinematics'][0], 2)
            pitch = round(drone.SENSOR_DATA['kinematics'][1], 2)
            yaw = round(drone.SENSOR_DATA['kinematics'][2], 2)

    return power, percentage, roll, pitch, yaw


def control_callback(data):
    global CMDS
    CMDS['roll'] = clamp(data.axis[0], STICK_MIN, STICK_MAX)
    CMDS['pitch'] = clamp(data.axis[1], STICK_MIN, STICK_MAX)
    CMDS['throttle'] = clamp(data.axis[2], 1000, THROTTLE_MAX)
    CMDS['yaw'] = clamp(data.axis[3], STICK_MIN, STICK_MAX)

    applyDeadZoneAndTrims()


def applyDeadZoneAndTrims():
    global CMDS
    # deadzone configuration

    input_range = 1000.0
    dead_zone = input_range * DEAD_ZONE
    roll, pitch, throttle, yaw = CMDS['roll'], CMDS['pitch'], CMDS['throttle'], CMDS['yaw']

    if abs(roll - 1500) < dead_zone:
        roll = 1500
    if abs(pitch - 1500) < dead_zone:
        pitch = 1500
    if abs(yaw - 1500) < dead_zone * 1.5:
        yaw = 1500
    if abs(throttle - 1000) < 50:
        throttle = 1000

    # scaling the sticks
    roll = 1500 + (roll - 1500) * STICK_SCALE
    pitch = 1500 + (pitch - 1500) * STICK_SCALE
    yaw = 1500 + (yaw - 1500) * STICK_SCALE

    CMDS['roll'], CMDS['pitch'], CMDS['throttle'], CMDS['yaw'] = int(roll), int(pitch), int(throttle), int(yaw)


def main(drone):
    rospy.init_node('Dispatcher')

    # subscribe to get controls axes from Interceptor
    rospy.Subscriber('Control', ControlAxesMsg, control_callback)

    # publish Attitude and Power info
    attitude_pub = rospy.Publisher('CraftAttitude', AttitudeMsg, queue_size=1)
    arm_pub = rospy.Publisher('Armed', Bool, queue_size=1)
    control_pub = rospy.Publisher('ControlSlow', ControlAxesMsg, queue_size=1)
    alt_pub = rospy.Publisher('Altitude', AltitudeMsg, queue_size=1)
    run_pub = rospy.Publisher('Run', RunInfoMsg, queue_size=1)

    drone.is_ser_open = not drone.connect(trials=drone.ser_trials)
    if drone.is_ser_open:
        rospy.loginfo("{}: Connected to FC on {}".format(
            rospy.get_caller_id(), serial_port))
    else:
        rospy.logerr("{}: Error opening serial port.".format(
            rospy.get_caller_id()))

    last_info = last_send = time.time()
    initial_altitude, _ = bmp.getAltAndTemp()
    ros_start_time = time.time()
    rate = rospy.Rate(20)  # Hz
    while not rospy.is_shutdown():

        armed = rospy.get_param("/run/armed")
        cam_angle = rospy.get_param("/physical/camera_angle")
        detection_started = rospy.get_param("/run/detection_started")

        arm_pub.publish(Bool(armed))

        # get board info
        power, percentage, roll, pitch, yaw = getFCinfo(drone)

        # arming and disarming
        CMDS['aux1'] = 2000 if armed else 1000

        # compensate camera angle pitch
        camera_angle = clamp(cam_angle + pitch, 0, 90)

        # setting the camera angle
        CMDS['aux2'] = round(1000 + (11 * camera_angle))

        # send the channels to the board
        if drone.send_RAW_RC([CMDS[i] for i in CMDS_ORDER]):
            dataHandler = drone.receive_msg()
            drone.process_recv_data(dataHandler)

        if time.time() - last_send > SEND_PERIOD:
            abs_alt, temp = bmp.getAltAndTemp()
            rel_alt = abs_alt - initial_altitude
            runtime = time.time() - ros_start_time

            if armed:
                control_pub.publish(ControlAxesMsg([CMDS['roll'], CMDS['pitch'], CMDS['throttle'], CMDS['yaw']]))

            alt_pub.publish(AltitudeMsg(rel_alt, abs_alt, temp))
            attitude_pub.publish(AttitudeMsg(roll, pitch, yaw, percentage, power, cam_angle))
            run_pub.publish(RunInfoMsg(runtime, detection_started))
            last_send = time.time()

        rate.sleep()

        # if time.time() - last_info > INFO_PERIOD:
        #     rospy.loginfo("{}: {:.0f}% left @ {:.0f}W, (R{:.2f}, P{:.2f}, Y{:.2f}) -> {}".format(
        #         rospy.get_caller_id(), percentage, power, roll, pitch, yaw, CMDS['throttle']))
        #     last_info = time.time()


if __name__ == '__main__':

    try:

        serial_port = "/dev/" + rospy.get_param("/serial/flight_controller")
        drone = MSPy(device=serial_port, loglevel="INFO", baudrate=115200)

        main(drone)

        drone.conn.close()

    except rospy.ROSInterruptException as error:
        pass
