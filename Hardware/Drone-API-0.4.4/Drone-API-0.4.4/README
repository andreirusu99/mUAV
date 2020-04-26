# **DroneAPI**

A Python package to control Drones that run the MultiWii Protocol. 

## **API Compatibility**

 

 - Drona Aviation Pluto 


## **Using the Library**

First install the package from pip

    pip install droneAPI

Import the Library in your Python Program

    import droneAPI

Create an object to the class droneAPI

    myDrone=droneAPI.droneAPI(IP, PORT)
The IP and the PORT parameters are specific to drones, but most common values are 192.168.4.1 and 23 respectively, for ESP8266 based Wifi Drones.

    myDrone=droneAPI.droneAPI('192.168.4.1', 23)
You can enable some basic debugging by adding a third parameter above, set to True

    myDrone=droneAPI.droneAPI('192.168.4.1', 23, True)

*If you experience lagging and erratic behavior, set the third variable to False or ignore it completely in the above statement.*  

## **Connecting and Arming The Drone**

Once a successful connection is made, you need to arm the drone. Most drones will arm with throttle set to 0%, to prevent the drone from flying immediately. 
Using the myDrone object, you can set throttle to 0% and arm the drone as follows. 

    myDrone.setThrottle(1000) //1000 by default is lowest range of the throttle
    myDrone.arm()

## **Flying the Drone**
The drone once armed, is ready to fly. 
**Caution: Please fly responsibly and outdoors, away from people and pets.**

**Increase Thottle**

The Throttle values on MultiWii systems range from the range [1000, 2000]. The 1000 value is the lower range. 
Get the drone to fly by setting a value here

    myDrone.setThrottle(1050)// Just spinning the rotors
     
or let it hover a little

    myDrone.setThrottle(1200)// Increase Throttle

**Change Pitch**

The Pitch values on MultiWii systems range from the range [1000, 2000]. The 1000 value is the lower range. 
Get the drone to Pitch forward and back by setting a value here

    myDrone.setPitch(1550)
     


**Change Roll**

The Roll values on MultiWii systems range from the range [1000, 2000]. The 1000 value is the lower range. 
Get the drone to Roll left and right back by setting a value here

    myDrone.setRoll(1300)

**Change Yaw**

The Yaw values on MultiWii systems range from the range [1000, 2000]. The 1000 value is the lower range. 
Get the drone to Yaw by setting a value here

    myDrone.setYaw(1200) 