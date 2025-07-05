#!/bin/bash

SESSION="navigation_real"

# Start new tmux session
tmux new-session -d -s $SESSION

# Window 0: Wheelchair Bringup (with chmod)
tmux rename-window -t $SESSION:0 "bringup"
sudo chmod 777 /dev/ttyACM0
sleep 5
tmux send-keys -t $SESSION:0 "ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=False" C-m

# Wait a bit before launching next nodes
sleep 5

# Window 1: Livox LiDAR
tmux new-window -t $SESSION -n "livox"
tmux send-keys -t $SESSION:1 "ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=False" C-m

# sleep 5

# Optional: Kinematic ICP
# tmux new-window -t $SESSION -n "kin_icp"
# tmux send-keys -t $SESSION:2 "ros2 launch kinematic_icp online_node.launch.py" C-m

# Optional: Realsense
# tmux new-window -t $SESSION -n "realsense"
# tmux send-keys -t $SESSION:3 "ros2 launch realsense2_camera rs_launch.py" C-m

# Optional: Keepout zones
# tmux new-window -t $SESSION -n "keepout"
# tmux send-keys -t $SESSION:4 "ros2 launch wheelchair2_navigation keepout.launch.py" C-m

sleep 5

# Window 2: Nav2
tmux new-window -t $SESSION -n "nav2"

# AMCL stack
tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=False map:=$MAP_DIR/rrc/rrc_lab.yaml" C-m
# tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=False map:=$MAP_DIR/himalaya/himalaya_with_classroom.yaml" C-m

# ICP stack
## for RRC lab
# tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2_icp.launch.py use_sim_time:=False map_file_path:=$MAP_DIR/rrc/rrc_lab.yaml pcd_map_path:=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd" C-m

## for Himalaya and KCIS
# tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2_icp.launch.py use_sim_time:=False map_file_path:=$MAP_DIR/himalaya/himalaya_with_classroom_ICP.yaml pcd_map_path:=$MAP_DIR/himalaya/himalaya_with_classroom_ICP.pcd" C-m

# Window 3: Teleop
tmux new-window -t $SESSION -n "teleop"
tmux send-keys -t $SESSION:3 "ros2 run wheelchair2_control keyboard_teleop_constant.py" C-m

# Optional: RViz
# tmux new-window -t $SESSION -n "rviz"
# tmux send-keys -t $SESSION:5 "rviz2 -d navigation.rviz" C-m

# Attach to session
tmux attach -t $SESSION
