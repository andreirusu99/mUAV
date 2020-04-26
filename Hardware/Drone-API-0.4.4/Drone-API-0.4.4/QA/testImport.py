import pyMultiWii
import time
myDrone=pyMultiWii.pyMultiWii("192.168.4.1", 23)
#myDrone.Connect()

myDrone.arm()
myDrone.setThrottle(1000)
myDrone.setThrottle(1050)
time.sleep(3)
myDrone.disarm()
myDrone.disconnect()
