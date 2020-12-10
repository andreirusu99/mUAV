# Hardware

## Quadcopter
Designed with 3D printing and modularity in mind, it acts as an enclosure for all the other components. 
Designed in SolidWorks and sliced with Ultimaker Cura.

It features a 3-deck design, where in the middle deck lie the most important components (Jetson Nano, Flight Controller, Camera). 
The upper deck hosts the GPS and the Wi-Fi antennas, while the lower deck contains the LiPo battery, the sonar, and a servo release mechanism.

The front arms are pulled back (in comparison to a regular simmetrical quadcopter) in order for them not to interfere with the camera's image. The propellers are placed well above the center of gravity for increased stability.
Additionally, each motor is inclined by 3 degrees horizontally to further improve hover stability.

The entire craft is suspended on 4 independent legs that can be easily replaced.
The motor arms are also modular and can be easily replaced if broken.
The sides are covered for extra protection in case of falls.

![Drone](https://i.imgur.com/v9RKz9P.jpeg)

## Main Computer
The nVidia Jetson Nano Developer Kit is used in this project, mainly due to the fact that it boasts a Maxwell GPU (alongside a 4 core ARM CPU) capable of fast deep-learning inference speeds.
Additionally, the wealth of expasion ports and headers helps in connecting sensors and building complex applications. 
The Jetson is improved by the addition of an Intel Wi-Fi card, in the dedicated E key slot under the Nano module. 
This card has 2 antennas, which come out of the top of the drone and give the nano very good wireless coverage.

Runs Tegra4Linux, a custom Linux distro specific to the Jetson platform.

![Jetson Nano](https://developer.nvidia.com/sites/default/files/akamai/embedded/images/jetsonNano/JetsonNano-DevKit_Front-Top_Right_trimmed.jpg)

## Flight Controller
This is the component that directly interacts with the motors. The FC contains 4 separates ESCs to control the 4 separate motors.
It also provides a pass-through interface to control the servo to which the camera is attached, for tilt action.
It receives MSP (MultiWii Serial Protocol) commands on separate channels and runs PID loops and stabilisation algorithms to keep the drone level.
Uses Betaflight firmware and has an integrated IMU with an accelerometer and gyroscope.
It is mounted in the head of the craft, behind the camera and the tilting servo.

![Flight Controller](https://cdn-global-hk.hobbyking.com/media/catalog/product/cache/1/image/660x415/17f82f742ffe127f42dca9de82fb58b1/2/0/202844_4.jpg)


## Sensors

### Camera
The camera used in this project is the RasberryPi Camera V2, with up to 4K resolution at 30FPS. It gives out a decent image, and has very good support on the Jetson Nano.
It plugs into one of the CSI camera ports on the Jetson.

![RPi Cam V2](https://www.raspberrypi.org/homepage-9df4b/static/5892e05a0858779e36ff6045dbc1a414/8924f/4275760945bb7f2b00766f92384de9124335995e_pi-camera-hero-1-1394x1080.jpg)

### Sonar
I used a simple HC-SR04(P) ultrasonic sensor as a sonar to obtain altitude information. It is mounted on the underside of the craft, facing downwards.

![Sonar](https://cdn.sparkfun.com//assets/parts/1/3/5/0/8/15569-Ultrasonic_Distance_Sensor_-_HC-SR04-01a.jpg)

### Barometer
A Bosch BMP-280 barometric and temperature sensor is used for altitude information when the craft is out of the range of the sonar (about 4m from the ground).
Communication between it and the Jetson is done using I2C. It is mounted behind the FC, above a 5V power regulator.

![BMP-280](https://images-na.ssl-images-amazon.com/images/I/41Di32vL6-L.jpg)

### GPS
As GPS, a VK-162 is used. It provides simple USB interface access and has good accuracy and reliability. 
It can be seen mounted on the topmost part of the drone, for better location signal.

![VK-162](https://images-na.ssl-images-amazon.com/images/I/31W6-Orz9qL._AC_.jpg)

## Actuators
### Camera tilt servo
The camera is mounted on a servo arm that allows its tilting from 0 to 90 degrees. This is a helpful feature to have when taking shots from above, and is commonly found on cinematic drones (although those have extra axes).

![Servo tilt](https://i.imgur.com/mPajbQS.png)

### Servo release mechaism
Additionally, a servo release mechanism is mounted on the underside of the craft, next to the sonar. It allows the carrying of objects, with a theoretical max capacity of about 2kg, but this is not tested.
> Note: this feature is not used in the project, I might do something with it afterwards

![Servo release](https://i.imgur.com/SF27KIW.png)


### Motors and Propellers
I used 4 Turnigy BLDC, 2600KV motors (common for medium-scale drones), together with 6040 propellers (6" long, 4" pitch), with a rated thrust of 800g each.

## Stats
- Mass without battery: **~850g**
- Mass with battery: **~1200g** (1x 3S LiPo 5200mAh, 55Wh)
- Propeller span: **51cm**
- Height: **15cm** (without antennas)
- Power consumption: **~10W** idle, **~150W** hover + **~10W** JetsonNano
- Average flight time: **20 min**
> Power consumption and flight times depend heavily on external factors such as air temperature, wind, humidity.

## Extra Photos
### Top View
> Shows the upper deck, with the Jetson Nano's fan, the GPS module, the antennas and various other design elements

![top](https://i.imgur.com/sxq4h51.jpeg)

### Bottom View
> Shos the sonar, the servo release, and the battery cage.

![bottom](https://i.imgur.com/uD0jeMe.jpeg)

### Back View
> Shows the I/O ports of the Jetson. Left USB is the Flight Controller, right USB is the GPS.

![back](https://i.imgur.com/8PYOLWV.jpeg)

### Flight Controller and Camera Servo
> You can spot the ventilation cutouts and the M3 nut capture locations embedded in the 3D-printed plastic

![front](https://i.imgur.com/JVggMCH.jpeg)

### Internal View
> Not all component had been installed when this picture was taken.

![internal](https://i.imgur.com/GOhfVe9.jpeg)
