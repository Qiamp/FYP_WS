#!/bin/bash

echo "========== 1. Starting VICON =========="
source /opt/ros/noetic/setup.bash
source /home/uav/FYP_WS/devel/setup.bash
roslaunch vrpn_client_ros sample.launch & sleep 6;

echo "========== 2. Starting mavros =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
sudo chmod 777 /dev/tty*
roslaunch mavros px4.launch & sleep 5;

echo "========== 3. Starting Offboard =========="
source /home/uav/FYP_WS/devel/setup.bash
roslaunch /home/uav/FYP_WS/src/my_offboard_node/offb.launch;

#echo "========== 4. Starting FAST LIO =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
#roslaunch fast_lio mapping_mid360_nrviz.launch & sleep 5;

#echo "========== 5. Starting Camera =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
#roslaunch realsense2_camera rs_t265.launch & sleep 5;

#echo "========== 6. Starting UWB =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
#roslaunch nlink_parser linktrack.launch & sleep 5;

wait;
