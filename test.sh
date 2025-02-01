#!/bin/bash

source /home/jay/FYP_WS/devel/setup.bash

roslaunch image_proc apriltag.launch & sleep 1;
roslaunch apriltag_ros continuous_detection.launch & sleep 1;
roslaunch apriltag_pose_pub apriltag_tracking.launch

