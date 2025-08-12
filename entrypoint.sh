#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/container_user/wheelchair2/install/setup.bash

# Wait until /dev/ttyACM0 exists
echo "Waiting for /dev/ttyACM0..."
while [ ! -e /dev/ttyACM0 ]; do
    sleep 1
    
done
echo "/dev/ttyACM0 detected."

SESSION="navigation_real"

# Start new tmux session
tmux new-session -d -s $SESSION

# Window 0: Wheelchair Bringup (with chmod)
tmux rename-window -t $SESSION:0 "bringup"
sudo chmod 777 /dev/ttyACM0
sleep 5
tmux send-keys -t $SESSION:0 "ros2 launch wheelchair2_bringup wheelchair2.launch.py use_ducky:=True use_sim_time:=False" C-m
sleep 5 

# Window 1: Livox LiDAR
tmux new-window -t $SESSION -n "livox"
tmux send-keys -t $SESSION:1 "ros2 launch wheelchair2_bringup livox.launch.py use_ducky:=True use_sim_time:=False" C-m

sleep 5

# Window 2: Nav2
tmux new-window -t $SESSION -n "nav2"

# AMCL stack
tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=False map:=$MAP_DIR/rrc/rrc_lab.yaml" C-m

# Window 3: Teleop
tmux new-window -t $SESSION -n "teleop"
tmux send-keys -t $SESSION:3 "ros2 run wheelchair2_control keyboard_teleop_constant.py" C-m

# Optional: RViz
 #tmux new-window -t $SESSION -n "rviz"
 #tmux send-keys -t $SESSION:5 "rviz2 -d navigation.rviz" C-m

# Attach to session

tmux attach -t $SESSION

