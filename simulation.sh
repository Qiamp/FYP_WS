#!/bin/bash


echo "========== 1. ROS Package Setup =========="
source ~/FYP_WS/devel/setup.bash

echo "========== 2. Starting Simulation Enviroment =========="
source ~/FYP_WS/Simulation/env_set.sh

echo "========== 2. Starting Custom gazebo =========="
roslaunch apriltag_tracking sitl_gazebo_custom.launch

echo "========== 3. Publishes camera_info =========="
python3 ~/FYP_WS/src/drone_apriltag_tracking_Landing/scripts/pub_camera_info.py ../config/gazebo_camera_cal.yaml 
