# colcon build --packages-select wheelchair2_navigation wheelchair2_control wheelchair2_bringup --symlink-install
# Set permissions for the Kangaroo motor controller
sudo chmod 777 /dev/ttyACM0

sleep 5

# Launch wheelchair bringup
xterm -e "ros2 launch wheelchair2_bringup wheelchair2_navigation.launch.py" &  

sleep 5

# # Launch 3D LiDAR and pointcloud to laserscan
# xterm -e "ros2 launch wheelchair2_bringup livox.launch.py" &

# sleep 5

# #launch kinematic icp
# # xterm -e "ros2 launch kinematic_icp online_node.launch.py" &

# # sleep 5

# #launch the realsense 
# # xterm -e "ros2 launch realsense2_camera rs_launch.py" &

# # sleep 5

# # Launch keepout zone
# xterm -e "ros2 launch wheelchair2_navigation keepout.launch.py" &

# sleep 5 

# Launch navigation stack
# xterm -e "ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=False" &

# sleep 5

# Launch RViz with the specified configuration
# xterm -e "rviz2 -d navigation.rviz" &

# sleep 5

xterm -e "ros2 run wheelchair2_control keyboard_teleop_constant.py" &
