#!/bin/bash

SESSION="navigation_real"

# Start new tmux session
tmux new-session -d -s $SESSION

# Window 0: Wheelchair Bringup
tmux rename-window -t $SESSION:0 "bringup"
sudo chmod 777 /dev/ttyACM0
sleep 5
tmux send-keys -t $SESSION:0 "ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=False" C-m

sleep 2

# Window 1: IMU (Adafruit)
tmux new-window -t $SESSION -n "IMU"
sudo chmod 666 /dev/gpiochip*
sudo chmod 666 /dev/i2c-7
tmux send-keys -t $SESSION:1 "ros2 run wheelchair2_localization adafruit_imu" C-m

sleep 2

# Window 2: Livox LiDAR
tmux new-window -t $SESSION -n "livox"
tmux send-keys -t $SESSION:2 "ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=False" C-m

sleep 2

# Window 3: RPLidar S2
tmux new-window -t $SESSION -n "rplidar"
sudo chmod 777 /dev/ttyUSB0
tmux send-keys -t $SESSION:3 "ros2 launch rplidar_ros rplidar_s2_launch.py" C-m

sleep 5

# Window 4: Robot Localization (EKF)
tmux new-window -t $SESSION -n "localization"
tmux send-keys -t $SESSION:4 "ros2 launch wheelchair2_localization robot_localization_launch.py" C-m

sleep 2

# Window 5: Nav2 (AMCL)
tmux new-window -t $SESSION -n "nav2"
# tmux send-keys -t $SESSION:5 "ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=False map:=$MAP_DIR/himalaya/himalaya_with_classroom.yaml" C-m

tmux send-keys -t $SESSION:5 "ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=False map:=$MAP_DIR/rrc/rrc_4th_floor.yaml" C-m

sleep 5

# Window 6: Rosbridge 
tmux new-window -t $SESSION -n "rosbridge"
tmux send-keys -t $SESSION:6 "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

sleep 2

# Window 7: GUI 
tmux new-window -t $SESSION -n "gui_controller"
tmux send-keys -t $SESSION:7 "ros2 run wheelchair_gui wheelchair_gui" C-m

# Attach to session
tmux attach -t $SESSION