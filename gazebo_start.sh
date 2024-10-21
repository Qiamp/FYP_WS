#!/bin/bash


echo "========== 1. ROS Package Setup =========="
source ~/FYP_WS/devel/setup.bash

echo "========== 2. Starting Simulation Enviroment =========="
source ~/FYP_WS/Simulation/env_set.sh

echo "========== 3. Starting Custom gazebo =========="
roslaunch apriltag_tracking sitl_gazebo_custom.launch

echo "========== 4. Publishes camera_info =========="
python3 ~/FYP_WS/src/drone_apriltag_tracking_Landing/scripts/pub_camera_info.py  ~/FYP_WS/src/drone_apriltag_tracking_Landing/config/gazebo_camera_cal.yaml
