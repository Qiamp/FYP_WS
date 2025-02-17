#!/bin/bash

source ~/FYP_WS/devel/setup.bash

echo "========== 3. Starting USB_CAM =========="
roslaunch usb_cam usb_cam-test.launch & sleep 5;

echo "========== 4. Starting Image_Proc =========="
roslaunch image_proc apriltag.launch & sleep 5;

echo "========== 5. Starting AprilTag_ROS =========="
roslaunch apriltag_ros continuous_detection.launch

wait;