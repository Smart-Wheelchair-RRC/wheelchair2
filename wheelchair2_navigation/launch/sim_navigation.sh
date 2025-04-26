
# Launch wheelchair bringup
xterm -e "ros2 launch wheelchair2_gazebo wheelchair2_sim.launch.py use_sim_time:=true use_joy:=true" &  

sleep 10

# # Launch 3D LiDAR and pointcloud to laserscan
xterm -e "ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=true" &

# sleep 5

# #launch kinematic icp
# # xterm -e "ros2 launch kinematic_icp online_node.launch.py" &

# # sleep 5

# #launch the realsense 
# # xterm -e "ros2 launch realsense2_camera rs_launch.py" &

# # sleep 5

# # Launch keepout zone
# xterm -e "ros2 launch wheelchair2_navigation keepout.launch.py" &

sleep 5 

# Launch navigation stack
xterm -e "ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=true map:=$MAP_DIR/rrc/himalaya_kcis.yaml" &

# sleep 5

# Launch RViz with the specified configuration
# xterm -e "rviz2 -d navigation.rviz" &

# sleep 5

# xterm -e "ros2 run wheelchair2_control keyboard_teleop_constant.py" &
