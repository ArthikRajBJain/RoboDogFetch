# RoboDog Fetch - Detection and Retrieving

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

* [/libraries](/libraries): Libraries for the whole Implementation.
