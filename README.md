# RoboDog Fetch - Detection and Retrieving

![RobotDog](/img/RoboDog.png)

Welcome to the Autonomous Tennis Ball Retrieval System repository! This project demonstrates an innovative robotic system that integrates object detection, distance estimation, and precise control to autonomously locate, approach, and retrieve a tennis ball. Designed for mobile manipulation tasks, the system utilizes advanced algorithms and libraries to achieve real-time performance.

## Features

* YOLOv8-Based Object Detection:
    * Fine-tuned model for accurate and real-time detection of tennis balls.
    * Utilizes Ultralytics and LabelImg for training and annotation.

* Distance Estimation via Curve Fitting:
    * Nonlinear mapping of bounding box size to object distance using Matlab.
    * Calibration performed for enhanced accuracy in varying scenarios.

* Inverse Kinematics (IK) Implementation:
    * Custom library for calculating the joint angles of a 3R planar arm.
    * Ensures precise manipulation of the robotic arm for object retrieval.
    
* Proportional Control System:
    * Smooth and adaptive control of the robot's movements and manipulator.
    * Ensures seamless integration of mobility and manipulation.

## Repository Structure

* [/robotFollow](/robotFollow): Final Implementation folder for Tennis Ball Detection and Grabing with the robot.
* [/ballGrab](/ballGrab): Implementation folder for Tennis Ball Detection and Grabing.
* [/trackingBall](/trackingBall): Implementation folder for Tennis Ball Detection and Tracking.
* [/libraries](/libraries): Libraries for the whole Implementation.
* [/libraries/dynamixelAPI](/libraries/dynamixelAPI): Library for the Dynamixel Motor and to move the OpenManipulator X Arm.
* [/libraries/dynamixelPIP](/libraries/dynamixelPIP): Low level library for the Dynamixel Motors from the manufacturer. 
* [/libraries/inverseKinematics](/libraries/inverseKinematics): Library for the Inverse Kinematics Operation.
* [/libraries/unitreeAPI](/libraries/unitreeAPI): Higher level Library for moving the Robot.
* [/libraries/unitreeSO](/libraries/unitreeSO): Dynamically linked Library for moving the Robot provided by the manufacturer.
* [/libraries/yoloDetection](/libraries/yoloDetection): Custom Trained YOLOv8 model for Tennis Ball detection.

## Installation

Installing Python 3.8, The dynamic library for moving the robot works only with **python3.8**
<br>
<br>
If using Fedora, RHEL Linux Distributions
<br>
```console
$ sudo dnf install python3.8
```
If using Ubuntu, Debian or Raspberry Pi OS Linux Distributions, apt doesn't have python3.8 hence compiling from source
<br>
```console
$ sudo apt install wget build-essential libreadline-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev curl llvm libncurses5-dev xz-utils liblzma-dev python3-openssl git
$ wget -c https://www.python.org/ftp/python/3.8.20/Python-3.8.20.tgz
$ tar -xf Python-3.8.20.tgz
$ cd Python-3.8.20/
$ ./configure --enable-optimizations
$ make -j$(nproc)
$ sudo make install
```
Creating a Virtual Environment and sourcing into it
<br>
```console
$ python3.8 -m venv venv
$ source venv/bin/activate
```
Installing Dynamixel library
<br>
```console
$ cd /libraries/dynamixelPIP/python/
$ python setup.py install
```
Installing Python Dependencies 
<br>
```console
$ pip install opencv-python labelImg ultralytics
```

## Running the Main Program

Make sure to use the correct Camera Interface for OpenCV and the tty port for communicating with the OpenManipulator X
```console
$ cd robotFollow
$ python robotFollow.py
```

## Author

- [Arthik Raj B Jain](https://github.com/ArthikRajBJain)



