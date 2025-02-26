#!/bin/bash

# Set permissions for the Kangaroo motor controller
sudo chmod 777 /dev/ttyACM0

sleep 5

# Launch wheelchair bringup
xterm -e "ros2 launch wheelchair2_bringup wheelchair2.launch.py in_simulation:=false" &

sleep 5

# Launch 3D LiDAR and pointcloud to laserscan
xterm -e "ros2 launch wheelchair2_bringup livox.launch.py" &

sleep 5

# Launch navigation stack
xterm -e "ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=False" &

sleep 5

# Launch RViz with the specified configuration
xterm -e "rviz2 -d navigation.rviz" &

sleep 5

# Launch RViz with the specified configuration
xterm -e "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" &
