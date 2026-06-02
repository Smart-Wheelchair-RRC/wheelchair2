#!/bin/bash


SESSION_NAME="wheelchair_navigation"
WHEELCHAIR_NAME="blacky"  # ducky
export DISPLAY=:1


################## 1. MAP PATHS
# uncomment the map you want to use for localization and navigation


## RRC with outside corridor
# PCD_PATH=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd
# YAML_PATH=$MAP_DIR/rrc/rrc_lab.yaml
# NATURE=indoor

## RRC with mech lab and outside corridor
# PCD_PATH=$MAP_DIR/rrc/hardware_lab_2/hardware_lab_2.pcd
# YAML_PATH=$MAP_DIR/rrc/hardware_lab_2/hardware_lab_2.yaml
# NATURE=indoor

## Himalaya with ramps
# PCD_PATH=$MAP_DIR/himalaya/himalaya_with_classroom_ICP.pcd
# YAML_PATH=$MAP_DIR/himalaya/rrc_outside_himalaya_ramps_fast_cleaned.yaml
# NATURE=outdoor

## 4th floor RRC
PCD_PATH=$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.pcd
YAML_PATH=$MAP_DIR/rrc/4th_floor_apr2026/2026_04_06.yaml
NATURE=indoor


# Start detached session
tmux new-session -d -s $SESSION_NAME -n "bringup" -x 200 -y 100

################## 2. WINDOW 0: BRINGUP
# Pane 0.0: Livox
tmux send-keys -t $SESSION_NAME:0 "ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True" C-m

# Pane 0.1: Controllers
tmux split-window -h -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0 "sudo chmod 777 /dev/ttyACM0 && sleep 2 && ros2 launch wheelchair2_bringup wheelchair2.launch.py robot_name:=$WHEELCHAIR_NAME use_sim_time:=False" C-m

# Pane 0.2: 2D LiDAR
tmux split-window -v -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME:0 "sudo chmod 777 /dev/ttyUSB1 && sleep 2 && ros2 launch clearance_sensors tof.launch.py" C-m

tmux split-window -v -t $SESSION_NAME:0.2
tmux send-keys -t $SESSION_NAME:0 "sudo chmod 777 /dev/ttyUSB0 && sleep 2 && ros2 launch wheelchair2_bringup rplidar.launch.py use_sim_time:=False" C-m

tmux select-layout -t $SESSION_NAME:0 tiled


################## 3. WINDOW 1: LOCALIZATION AND NAVIGATION
tmux new-window -t $SESSION_NAME -n "loc_nav"

# Pane 1.0: Localization
tmux send-keys -t $SESSION_NAME:1 "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=$NATURE | tee loc.log" C-m

# Pane 1.1: Navigation
tmux split-window -h -t $SESSION_NAME:1
tmux send-keys -t $SESSION_NAME:1 "sleep 7 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$YAML_PATH | tee nav.log" C-m

# Pane 1.2: Click Loc (Optional/Background)
# tmux split-window -v -t $SESSION_NAME:1
# tmux send-keys -t $SESSION_NAME:1 "python3 /home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py" 

tmux select-layout -t $SESSION_NAME:1 tiled


################## 4. WINDOW 2: APP AND GUI
tmux new-window -t $SESSION_NAME -n "app"

# Pane 2.0: ROS Bridge / GUI
tmux send-keys -t $SESSION_NAME:2 "ros2 launch wheelchair2_gui wheelchair_gui.launch.py | tee gui.log" C-m

# Pane 2.1: Extra (e.g., Teleop or Monitoring)
# tmux split-window -h -t $SESSION_NAME:2
# tmux send-keys -t $SESSION_NAME:2 "ros2 run wheelchair2_control keyboard_teleop_constant.py"

# tmux select-layout -t $SESSION_NAME:2 tiled


################## ATTACH
# Start the user on the Bringup window
tmux select-window -t $SESSION_NAME:0
tmux attach-session -t $SESSION_NAME