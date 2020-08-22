#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from drone.msg import ControlAxes
from drone.msg import Attitude
from drone.msg import Power

from yamspy import MSPy
import time, threading, os

FCinfo = ['MSP_ANALOG', 'MSP_ATTITUDE']

CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 1000,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

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


def callback(data, drone):

    CMDS['roll'] = data.data[0]
    CMDS['pitch'] = data.data[1]
    CMDS['throttle'] = data.data[2]
    CMDS['yaw'] = data.data[3]
    
    
def main(drone):

    rospy.init_node('Dispatcher')
    
    # subscribe to get control axes from Interceptor
    rospy.Subscriber('Control', ControlAxes, callback, drone)

    # publish attitude and power info
    attitude_pub = rospy.Publisher('CraftAttitude', Attitude, queue_size = 2)
    power_pub = rospy.Publisher('CraftPower', Power, queue_size = 2)

    drone.is_ser_open = not drone.connect(trials = drone.ser_trials)
    if drone.is_ser_open :
        rospy.loginfo("{}: Connected to FC on {}".format(rospy.get_caller_id(), serial_port))
    else :
        rospy.logerr("{}: Error opening serial port.".format(rospy.get_caller_id()))
        os._exit(1)

    while not rospy.is_shutdown():

        armed = rospy.get_param("/run/armed")

        CMDS['aux1'] = 2000 if armed else 1000

        # send the channels to the board
        if(drone.send_RAW_RC([CMDS[i] for i in CMDS_ORDER])):
            dataHandler = drone.receive_msg()
            drone.process_recv_data(dataHandler)

        # get board info
        power, percentage, roll, pitch, yaw = getFCinfo(drone)
        attitude_pub.publish(Attitude(roll, pitch, yaw))
        power_pub.publish(Power(percentage, power))

        rospy.loginfo("{:.0f}% left @ {:.0f}W, ({:.2f} R {:.2f} P {:.2f} Y)".format(percentage, power, roll, pitch, yaw))


if __name__ == '__main__':

    try:

        serial_port = "/dev/" + rospy.get_param("/board/serial_port")
        drone = MSPy(device = serial_port, loglevel = "INFO", baudrate = 115200)
            
        main(drone)

    except rospy.ROSInterruptException as error:
        drone.conn.close()