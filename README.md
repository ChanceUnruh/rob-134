# FaceTrackerROS
![Python](https://img.shields.io/badge/Python-3.x-blue) 
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange) 
![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)

A ROS (Robot Operating System) node that uses OpenCV to detect faces in a webcam feed and controls a TurtleSim robot to track the detected face by mapping its position to the TurtleSim environment.

## Overview

`FaceTrackerROS` is a Python-based ROS node that:
1. Subscribes to a webcam image topic (`/usb_cam_node/image_raw`).
2. Detects human faces in the video feed using OpenCV's Haar Cascade Classifier.
3. Maps the detected face's center to the TurtleSim coordinate system (11x11 grid).
4. Publishes velocity commands (`/turtle1/cmd_vel`) to move the TurtleSim turtle toward the mapped face position.
5. Displays the processed video feed with bounding boxes and center points overlaid on detected faces.

This project integrates computer vision with robotic control, demonstrating a simple face-tracking application using ROS and TurtleSim.


## Prerequisites

- **Ubuntu** (tested on 20.04 or later recommended)
- **ROS** (Noetic or later recommended)
- **Python 3.x**
- **OpenCV** (`cv2`) with Haar Cascade support
- **ROS Packages:**
  - `rospy`
  - `sensor_msgs` (for `Image` message type)
  - `geometry_msgs` (for `Twist` and `Point` message types)
  - `turtlesim` (for `Pose` message type and simulation)
  - `cv_bridge` (for converting ROS images to OpenCV format)
  - `usb_cam` (for webcam support)
- **USB Webcam** (or another ROS-compatible camera publishing to `/usb_cam_node/image_raw`)

## Installation

1. **Set up your ROS environment:**
   Install ROS Noetic and create a Catkin Workspace by following the official guide at [ROS Noetic Installation on Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu).

## Create Package  

Once you have installed ROS Noetic and created a Catkin Workspace, you can create the following package:

1. **Navigate to Your Catkin Workspace**  
   ```bash
   cd ~/catkin_ws/src
   ```

2. **Create the Package**  
   ```bash
   catkin_create_pkg rob_134 rospy turtlesim
   ```

3. **Build the Workspace**  
   ```bash
   cd ~/catkin_ws
   ```

4. **Build the Package**  
   ```bash
   catkin_make
   ```

5. **Source the Workspace**  
   ```bash
   source devel/setup.bash
   ```

Your package `rob_134` is now ready to use!



    
