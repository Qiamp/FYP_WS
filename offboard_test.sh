#!/bin/bash

echo "========== 1. Starting VICON =========="
source /opt/ros/noetic/setup.bash
source ~/FYP_WS/devel/setup.bash
roslaunch vrpn_client_ros sample.launch & sleep 6;

echo "========== 2. Starting mavros =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
sudo chmod 777 /dev/tty*
roslaunch mavros px4.launch & sleep 5;

echo "========== 3. Starting Offboard =========="
source ~/FYP_WS/devel/setup.bash
roslaunch ~/FYP_WS/src/offboard_landing/launch/offb.launch;



wait;
