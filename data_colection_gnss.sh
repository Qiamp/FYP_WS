#!/bin/bash

echo "========== 1. Starting ntrip_client =========="
source /opt/ros/noetic/setup.bash
source ~/FYP_WS/devel/setup.bash
roslaunch ntrip_client ntrip_client.launch& sleep 6;

echo "========== 2. Starting mavros =========="
sudo chmod 777 /dev/tty*
roslaunch mavros test.launch & sleep 5;

echo "========== 3. Starting USB_CAM =========="
roslaunch usb_cam usb_cam-test.launch & sleep 5;

echo "========== 4. Starting Image_Proc =========="
roslaunch image_proc apriltag.launch & sleep 5;

echo "========== 5. Starting AprilTag_ROS =========="
roslaunch apriltag_ros continuous_detection.launch & sleep 5;

echo "========== 6. Starting AprilTag_Pose_Pub =========="
roslaunch apriltag_pose_pub apriltag_tracking.launch & sleep 60;

# rosservice call /mavros/cmd/command "{  
#   command: 246,               # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN  
#   confirmation: 0,  
#   param1: 1,                  # 1 = 重启飞控  
#   param2: 0, param3: 0, param4: 0,  
#   param5: 0, param6: 0, param7: 0  
# }"


# echo "========== 6. Starting Offboard Flight ==========" -6.51 -0.48
# source ~/FYP_WS/devel/setup.bash
# roslaunch offboard_landing_gnss offb.launch

# source ~/FYP_WS/devel/setup.bash
# rosbag record -a
# Modify

wait;
