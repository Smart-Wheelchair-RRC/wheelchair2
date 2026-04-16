#!/bin/bash

# 0. Hardware Permissions
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyUSB0
sudo chmod 666 /dev/gpiochip*
sudo chmod 666 /dev/i2c-7
sleep 2

SESSION_NAME="wheelchair_navigation"

# Path Setup
MAP_DIR="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps"
PCD_PATH="$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.pcd"
YAML_PATH="$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.yaml"



# Kill old sessions
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start session with large virtual size to ensure all 7 panes fit
tmux new-session -d -s $SESSION_NAME -x 200 -y 100

############## 2. Pane 0: Wheelchair2 bringup
tmux send-keys -t $SESSION_NAME:0.0 "ros2 launch wheelchair2_bringup wheelchair2.launch.py robot_name:=ducky" C-m
##############

############## 3. Pane 1: livox.launch.py
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled
tmux send-keys -t $SESSION_NAME:0.1 "ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True" C-m
##############

############## 4. Pane 2: Localisation launch
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled
tmux send-keys -t $SESSION_NAME:0.2 "python3 /home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py &" C-m

# (Default) RRC with outside corridor
# tmux send-keys -t $SESSION_NAME:0.2 "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=indoor | tee ii.log" C-m 

# RRC with mech lab and outside corridor
tmux send-keys -t $SESSION_NAME:0.2 "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.pcd nature:=indoor use_imu:=false" C-m

# Himalaya with ramps
# tmux send-keys -t $SESSION_NAME:0.2 "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/himalaya/himalaya_with_classroom_ICP.pcd nature:=outdoor" C-m
##############

############## 5. Pane 3: sleep 10s then nav2_3d.launch.py
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled

# (Default) RRC with outside corridor
# tmux send-keys -t $SESSION_NAME:0.3 "sleep 10 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$YAML_PATH | tee output.log" C-m

# RRC with mech lab and outside corridor
tmux send-keys -t $SESSION_NAME:0.3 "sleep 10 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.yaml | tee ~/wheelchair2/src/outp.log" C-m

# Himalaya with ramps
# tmux send-keys -t $SESSION_NAME:0.3 "sleep 10 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/himalaya/rrc_outside_himalaya_ramps_fast_cleaned.yaml | tee output.log" C-m
##############

############## 6. Pane 4: RPLidar
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled
tmux send-keys -t $SESSION_NAME:0.4 "ros2 launch rplidar_ros rplidar_s2_launch.py" C-m
##############


############## 7. Rosbridge + GUI Launch
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled
tmux send-keys -t $SESSION_NAME:0.5 "sleep 5 && ros2 launch wheelchair_gui wheelchair_gui.launch.py" C-m
##############

tmux select-layout -t $SESSION_NAME tiled
tmux attach-session -t $SESSION_NAME