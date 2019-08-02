#!/usr/bin/env -i bash

source /opt/ros/melodic/setup.bash
cd /home/lucius/Projects/PX4_Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

WORLD_FILE_NAME=$(pwd)/Tools/sitl_gazebo/worlds/landing.world

exec roslaunch px4 multi_uav_mavros_sitl_sdf.launch world:=${WORLD_FILE_NAME} vehicle:=iris_mono_cam