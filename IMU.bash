echo "========== 1. Starting mavros =========="
#source /home/nvidia/ws_uav_setup/devel/setup.bash
sudo chmod 777 /dev/tty*
roslaunch mavros calibrate.launch;
# rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0