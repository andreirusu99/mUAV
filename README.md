### ***This project represents the work for my bachelor's thesis and therefore, any commercial usage or distribution of the files without expressly granted written permission and consent is prohibited.***

Complete information in the [Thesis](/Thesis.pdf) document and the presentation (_Crowd identification and monitoring using UAVs.pdf_)

## Summary
Edge computing system for mUAV crowd identification and monitoring, complete with client application, real-time dashboard and statistics.

The purpouse of the project is to implement a crowd identification and monitoring system that runs on edge, on a drone that I designed and built (see *Hardware* folder for details).

Edge computing is done on a nVIDIA Jetson Nano Dev Kit installed on the quadcopter, that runs a ROS environment.

Crowd monitoring is performed using neural networks designed for edge devices (SSD MobileNet V2) and computer vision frameworks such as OpenCV.

The Jetson communicates completely wireless over a Wi-Fi hotspot from a computer (my laptop).

## Frameworks & APIs
- **ROS Melodic** (Robot Operating System), Python implementation
- **OpenCV 4.1.1**, built from source with CUDA support
- **TensorFlow 2**, for model tests
- **Flask Server**, for live video streaming
- **ReactJS**, for the front-end application
- **GoogleMapsAPI** for real-time GPS localization
- **ROSLibJS** for remote ROS topic access
- **YAMSPy** for MSP communication with the FC

## Folder Structure
### [catkin_ws](/catkin_ws)
Contains the ROS workspace, which constitutes the main functional part of the system. Inside, there are the source files of ROS Nodes, message definitions, and a launch file that launches all the required nodes. The programming language is Python, using the *rospy* library.

### [ros_client](/ros_client)
Contains a React application that uses ROSLibJS to interact with the Master Node running on the Jetson Nano, that displays real-time data in a dashboard by accessing the topics of the ROS runtime. 

### [GroundStation](/GroundStation)
Contains a Python script used to send UDP commands over Wi-Fi from a joystick connected to the ground computer (my laptop). The commands are sent as-is and all processing and interpretation of these commands is done in the air on the Jetson.
> Note: the drone's flight is not automated, therefore an operator is required for flying (flight automation was outside the scope of this project).

### [Hardware](/Hardware)
Contains the 3D models that make up the quadcopter. These parts were designed with 3D printing manufacturing in mind, and as such, are only suitable for this kind of manufacturing process. The parts were printed on my own 3D printer using PETG. and PLA. A detailed description of all the hardware components used (boards, sensors, actuators) is found in the Readme file inside the folder.

### [ModelTest](/ModelTest)
Is a folder that I created to host my neural network experiments, until I decied for the final implementation that I would transfer over to the Jetson for inferencing.

### Further information can be found in each respective folder.

## Helpful work by others
 - [DronePilot](https://github.com/alduxvm/DronePilot) - UDP joystick communication between computers
 - [YAMSPy](https://github.com/thecognifly/YAMSPy) - MSP library for Python
 - [Jetson Inference](https://github.com/dusty-nv/jetson-inference) - official deep learning inference library from nVidia
