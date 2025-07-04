#!/bin/bash

SESSION="navigation_sim"

# Start new tmux session
tmux new-session -d -s $SESSION

# Window 1: Gazebo Simulation
tmux rename-window -t $SESSION:0 'gazebo_sim'
tmux send-keys -t $SESSION:0 'ros2 launch wheelchair2_gazebo wheelchair2_sim.launch.py use_sim_time:=true use_joy:=true' C-m

# Wait a bit before launching next nodes
sleep 10

# Window 2: Livox LiDAR
tmux new-window -t $SESSION -n 'livox_lidar'
tmux send-keys -t $SESSION:1 'ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=true' C-m

sleep 5

# Window 3: Navigation
tmux new-window -t $SESSION -n 'nav2'
tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=true map:=$MAP_DIR/sim/obstacles_world.yaml" C-m

# (Optional) Window 4: Keepout Zones
# tmux new-window -t $SESSION -n 'keepout'
# tmux send-keys -t $SESSION:3 'ros2 launch wheelchair2_navigation keepout.launch.py' C-m

# Window 6: Keyboard Teleop
tmux new-window -t $SESSION -n 'teleop'
tmux send-keys -t $SESSION:3 'ros2 run wheelchair2_control keyboard_teleop_constant.py' C-m

# Attach to session
tmux attach -t $SESSION
