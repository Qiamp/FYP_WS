#!/bin/bash

echo "========== 1. ROS Package Setup =========="
source ~/FYP_WS/devel/setup.bash


echo "========== 2. image_proc launch =========="
export ROS_NAMESPACE=/iris/usb_cam
roslaunch apriltag_tracking image_proc_gazebo.launch & sleep 5

echo "========== 2. AprilTag_ROS =========="
export ROS_NAMESPACE=/iris/usb_cam
roslaunch apriltag_tracking gazebo_detection.launch