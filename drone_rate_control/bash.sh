#!/bin/bash
source /home/chasing/drone_new/devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/chasing/project/some/src/simulation/models
source /home/chasing/project/some/Firmware/Tools/setup_gazebo.bash /home/chasing/project/some/Firmware /home/chasing/project/some/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/chasing/project/some/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/chasing/project/some/Firmware/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/chasing/drone_new/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/chasing/project/some/src
