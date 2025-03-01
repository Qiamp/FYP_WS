#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/FYP_WS/devel/setup.bash


echo "========== 1. Starting mavros =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
sudo chmod 777 /dev/tty*
roslaunch mavros px4.launch & sleep 5;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0

echo "========== 2. Starting USB_CAM =========="
roslaunch usb_cam usb_cam-test.launch;