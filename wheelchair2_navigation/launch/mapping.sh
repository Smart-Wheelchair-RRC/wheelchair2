#!/bin/bash

# Set permissions for the Kangaroo motor controller
sudo chmod 777 /dev/ttyACM0
#dssds
sleep 5

# Launch wheelchair bringup
xterm -e "ros2 launch wheelchair2_bringup wheelchair2.launch.py in_simulation:=false" &

sleep 5

# Launch 3D LiDAR and pointcloud to laserscan
xterm -e "ros2 launch wheelchair2_bringup livox.launch.py" &

sleep 5

xterm -e "ros2 launch realsense2_camera rs_launch.py" &

sleep 5

#rtabmap
xterm -e "ros2 launch wheelchair2_navigation rtabmap_lidar_camera_icp.launch.py" &

# sleep 5

# keyboard teleop
xterm -e "ros2 run wheelchair2_control keyboard_teleop.py" &
