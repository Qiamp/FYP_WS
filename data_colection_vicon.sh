#!/bin/bash

echo "========== 1. Starting VICON =========="
# source /opt/ros/noetic/setup.bash
source ~/FYP_WS/devel/setup.bash
roslaunch vrpn_client_ros sample.launch & sleep 6;

echo "========== 2. Starting mavros =========="
sudo chmod 777 /dev/tty*
roslaunch mavros px4.launch & sleep 5;
# rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
# 175HZ

echo "========== 3. Starting USB_CAM =========="
roslaunch usb_cam usb_cam-test.launch & sleep 5;

echo "========== 4. Starting Image_Proc =========="
roslaunch image_proc apriltag.launch & sleep 5;

echo "========== 5. Starting AprilTag_ROS =========="
roslaunch apriltag_ros continuous_detection.launch & sleep 5;

echo "========== 6. Starting AprilTag_Pose_Pub =========="
roslaunch ~/FYP_WS/src/apriltag_pose_pub/launch/apriltag_tracking.launch

# source ~/FYP_WS/devel/setup.bash
# rostopic echo /tag_detections/inertial_pose
# rosbag record -a
# Modify

wait;
