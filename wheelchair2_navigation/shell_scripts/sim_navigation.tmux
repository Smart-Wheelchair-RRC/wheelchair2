#!/bin/bash

SESSION="navigation_sim"

# Start new tmux session
tmux new-session -d -s $SESSION

# Window 0: Gazebo Simulation
tmux rename-window -t $SESSION:0 'gazebo_sim'
tmux send-keys -t $SESSION:0 'ros2 launch wheelchair2_gazebo wheelchair2_sim.launch.py use_sim_time:=true use_joy:=true' C-m

# Wait a bit before launching next nodes
sleep 10

# Window 1: Livox LiDAR
tmux new-window -t $SESSION -n 'livox_lidar'
tmux send-keys -t $SESSION:1 'ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=true' C-m

sleep 5

# Window 2: Navigation
tmux new-window -t $SESSION -n 'nav2'
tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=true map:=$MAP_DIR/sim/obstacles_world.yaml" C-m

# (Optional) Window 3: Keepout Zones
# tmux new-window -t $SESSION -n 'keepout'
# tmux send-keys -t $SESSION:3 'ros2 launch wheelchair2_navigation keepout.launch.py' C-m

# Window 3: FAST-LIO
tmux new-window -t $SESSION -n 'fast_lio'
tmux send-keys -t $SESSION:3 'ros2 launch fast_lio mapping.launch.py use_sim_time:=true config_file:=mid360.yaml rviz:=false config_path:=/home/wheelchair2/wheelchair2/src/dependencies/FAST_LIO_ROS2/config' C-m

sleep 5

# Window 4: Keyboard Teleop
tmux new-window -t $SESSION -n 'teleop'
tmux send-keys -t $SESSION:4 'ros2 run wheelchair2_control keyboard_teleop_constant.py' C-m

# Attach to session
tmux attach -t $SESSION
