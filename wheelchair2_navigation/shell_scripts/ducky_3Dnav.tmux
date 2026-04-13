#!/bin/bash

# 0. Hardware Permissions
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyUSB0
sleep 2

SESSION_NAME="wheelchair_navigation"

# Path Setup
MAP_DIR="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps"
PCD_PATH="$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.pcd"
YAML_PATH="$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.yaml"

# File checks
if [ ! -f "$PCD_PATH" ]; then echo "ERROR: PCD file not found at $PCD_PATH"; fi
if [ ! -f "$YAML_PATH" ]; then echo "ERROR: YAML file not found at $YAML_PATH"; fi

# Kill old sessions
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start session with large virtual size to ensure all 7 panes fit
tmux new-session -d -s $SESSION_NAME -x 200 -y 100
##############

############## 2. Pane 0: Wheelchair2 bringup
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_bringup wheelchair2.launch.py robot_name:=ducky" C-m
##############

############## 3. Pane 1: livox.launch.py
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True" C-m
##############

############## 4. Pane 2: Localisation launch
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "python3 /home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py &" C-m

# (Default) RRC with outside corridor
# tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=indoor | tee ii.log" C-m 

# RRC with mech lab and outside corridor
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.pcd nature:=indoor" C-m

# Himalaya with ramps
# tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$MAP_DIR/himalaya/himalaya_with_classroom_ICP.pcd nature:=outdoor" C-m
##############

############## 5. Pane 3: sleep 10s then nav2_3d.launch.py
tmux split-window -t $SESSION_NAME
# (Default) RRC with outside corridor
# tmux send-keys -t $SESSION_NAME "sleep 10 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$YAML_PATH | tee output.log" C-m

# RRC with mech lab and outside corridor
tmux send-keys -t $SESSION_NAME "sleep 10 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.yaml | tee ~/wheelchair2/src/outp.log" C-m

# Himalaya with ramps
# tmux send-keys -t $SESSION_NAME "sleep 10 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$MAP_DIR/himalaya/rrc_outside_himalaya_ramps_fast_cleaned.yaml | tee output.log" C-m
##############

############## 6. Pane 4: RPLidar
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "ros2 launch rplidar_ros rplidar_s2_launch.py" C-m
##############

############## 7. Pane 5: Rosbridge 
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m
##############

############## 8. Pane 6: GUI 
tmux split-window -t $SESSION_NAME
# Added a small sleep and export DISPLAY to help the GUI launch correctly in the pane
tmux send-keys -t $SESSION_NAME "sleep 2 && export DISPLAY=$DISPLAY && ros2 run wheelchair_gui wheelchair_gui" C-m
##############

############## 9. Arrange all 7 panes in a tiled layout
tmux select-layout -t $SESSION_NAME tiled
##############

############## 10. Attach to the session to view the output
tmux attach-session -t $SESSION_NAME
##############