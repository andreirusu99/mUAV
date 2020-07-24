#!/usr/bin/env python

""" 
Command Interceptor (Air)

Acts as a bridge between the GCS and the air vehicle.

-> Reads the data coming from the GCS over UDP
-> Maps Joystick axis values to PWM values
-> Filters and applies other transformations to the values (dead-zone etc.)
-> Constructs commands from the received data (button presses etc.)
-> Passes the new information forward to the Command Router (RC control + commands)

"""
import rospy
import time, threading, os
import UDPserver as udp

from std_msgs.msg import String

_TAG = "Interceptor"


def mainThread():
    _TAG = "Main Thread"
    
    # rospy.loginfo(time.ctime(), _TAG, "Started")
    try:

        udp.startTwisted()

        while True:

            current = time.time()
            elapsed = 0

            # if udp.active:

            #     last_active = time.time()

            #     rospy.loginfo(udp.message)
            #     print(udp.message)

                
            # else: # udp not active
            #     rospy.loginfo(time.ctime(), _TAG, "UDP timeout")
            #     # connection lost to Ground Station

            while elapsed < 0.01:
                    elapsed = time.time() - current
    
    except Exception as error:
        os._exit(1)


if __name__ == "__main__":

    try:
        thread_main = threading.Thread(target = mainThread)
        thread_main.daemon = True
        thread_main.start()

        pub = rospy.Publisher('smth', String, queue_size=2)
        rospy.init_node('InterceptorNode')
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pub.publish(str(udp.message))
            rospy.loginfo("OK")
            rate.sleep()
        

    except Exception as error:
        rospy.loginfo("ERROR on main")
        os._exit(1)

    except KeyboardInterrupt:
        os._exit(1)