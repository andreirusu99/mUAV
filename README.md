# mUAV

## Summary
Repository containing the code for my bachelor's thesis.

The purpouse of the project is to implement a crowd monitoring system that runs on edge, on a drone that I designed and built (see *Hardware* section).

Edge computing is done on a nVIDIA Jetson Nano Dev Kit installed on the quadcopter, that runs a ROS environment of about 10 nodes.

Crowd monitoring is performed using neural networks designed for edge devices (SSD MobileNet V2) and computer vision frameworks such as OpenCV.

## Folder Structure
### catkin_ws
Contains the ROS workspace, which constitutes the main functional part of the system. Inside, there are the source files of ROS Nodes, message definitions, and a launch file that launches all the required nodes. The programming language is Python, using the *rospy* library.

### ros_client
Contains a React application that uses ROSLibJS to interact with the Master Node running on the Jetson Nano, that displays real-time data in a dashboard by accessing the topics of the ROS runtime. 

### GroundStation
Contains a Python script used to send UDP commands from a joystick connected to the ground computer (my laptop). The commands are sent as-is and all processing is done on the Jetson.
> Note: the drone's flight is not automated, therefore and operator is required for flying (flight automation was outside the scope of this project).

### Hardware
Contains the 3D models that make up the quadcopter. These parts were designed with 3D printing manufacturing in mind, and as such, are only suitable for this kind of manufacturing process. The parts were printed on my own 3D printer using PETG. and PLA. A detailed description of all the hardware components used (boards, sensors, actuators) is found in the Readme file inside the folder.

### ModelTest
Is a folder that I created to host my neural network experiments, until I decied for the final implementation that I would transfer over to the Jetson for inferencing.

#### Further information can be found in each respective folder.
