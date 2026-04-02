#!/bin/bash

export DISPLAY=:1
sudo chmod 777 /dev/ttyACM0
sleep 2

SESSION_NAME="wheelchair_navigation"


############## 1. Create a new tmux session in detached mode (This creates Pane 0)
tmux new-session -d -s $SESSION_NAME
##############


############## 2. Pane 0: Wheelchair2 bringup
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_bringup wheelchair2.launch.py" C-m
##############


############## 3. Pane 1: livox.launch.py
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True" C-m
##############


############## 4. Pane 2: Localisation launch
tmux split-window -t $SESSION_NAME

# Launch the Click Loc button 
# tmux send-keys -t $SESSION_NAME "python3 /home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py &" C-m


# (Default) RRC with outside corridor
# tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd nature:=indoor" C-m

# RRC with mech lab and outside corridor
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/rrc/hardware_lab_2/hardware_lab_2.pcd nature:=indoor" C-m

# Himalaya with ramps
# tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/himalaya/himalaya_with_classroom_ICP.pcd nature:=outdoor" C-m
##############


############## 5. Pane 3: sleep 7s then nav2_3d.launch.py
tmux split-window -t $SESSION_NAME

# (Default) RRC with outside corridor
# tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/rrc/rrc_lab.yaml nav2_params_file:=/home/container_user/wheelchair2/src/wheelchair2_navigation/config/merge_params.yaml | tee /home/container_user/wheelchair2/src/output.log" C-m

# RRC with mech lab and outside corridor
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/rrc/hardware_lab_2/hardware_lab_2.yaml nav2_params_file:=/home/container_user/wheelchair2/src/wheelchair2_navigation/config/merge_params.yaml | tee /home/container_user/wheelchair2/src/output.log" C-m

# Himalaya with ramps
# tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/himalaya/rrc_outside_himalaya_ramps_fast_cleaned.yaml | tee output.log" C-m
##############


############## 6. Arrange the 4 panes in a tiled layout
tmux select-layout -t $SESSION_NAME tiled
##############


############## 7. Attach to the session to view the output
tmux attach-session -t $SESSION_NAME
##############