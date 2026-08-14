#!/bin/bash
export DISPLAY=:1001

sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.wmem_max=10485760

# Make it permanent
echo "net.core.rmem_max=10485760" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=10485760" | sudo tee -a /etc/sysctl.con

# Source setup paths
# colcon build --symlink-install --packages-select wheelchair2_navigation

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_SECURITY_ENABLE=false
export ROS_DOMAIN_ID=56

# Use CycloneDDS to match the Jetson's RMW
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=file:///home/robot/wheelchair_ws/wheelchair2/scripts/cyclonedds_laptop.xml
export CYCLONEDDS_URI=file:///home/container_user/wheelchair2/src/scripts/cyclonedds_jetson.xml

# Launch RViz
ros2 launch wheelchair2_navigation rviz_nav.launch.py
