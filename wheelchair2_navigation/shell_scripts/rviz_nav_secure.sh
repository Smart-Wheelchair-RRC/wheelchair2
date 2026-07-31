#!/bin/bash

# Build navigation package on laptop
colcon build --symlink-install --packages-select wheelchair2_navigation wheelchair_intelligence_ros2

# Source setup paths
source /opt/ros/humble/setup.bash
source install/setup.bash

# Enable SROS2 Security
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/home/robot/.ros/sros2/keystore
export ROS_SECURITY_ENCLAVE=/wheelchair
export ROS_DOMAIN_ID=56

# Use CycloneDDS to match the Jetson's RMW
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/robot/wheelchair_ws/wheelchair2/scripts/cyclonedds_laptop.xml

# Launch RViz
ros2 launch wheelchair2_navigation rviz_nav.launch.py
