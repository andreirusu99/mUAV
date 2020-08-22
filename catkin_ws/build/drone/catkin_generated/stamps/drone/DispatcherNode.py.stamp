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


def callback(data, drone):

    CMDS['roll'] = data.data[0]
    CMDS['pitch'] = data.data[1]
    CMDS['throttle'] = data.data[2]
    CMDS['yaw'] = data.data[3]
    
    
def main(drone):

    rospy.init_node('Dispatcher')
    
    rospy.Subscriber('Control', ControlAxes, callback, drone)
    rospy.Publisher('CraftAttitude', Attitude, queue_size = 2)
    rospy.Publisher('CraftPower', Power, queue_size = 2)

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
            

if __name__ == '__main__':

    try:

        serial_port = "/dev/" + rospy.get_param("/board/serial_port")
        drone = MSPy(device = serial_port, loglevel = "INFO", baudrate = 115200)
            
        main(drone)

    except rospy.ROSInterruptException as error:
        drone.conn.close()