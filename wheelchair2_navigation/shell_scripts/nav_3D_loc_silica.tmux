#!/bin/bash

SESSION_NAME="wheelchair_navigation"

tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Session $SESSION_NAME already exists. Exiting to prevent duplicate nodes."
    exit 0
fi

WHEELCHAIR_NAME="ducky"  # ducky
export DISPLAY=:1001

# RRC with mech lab and outside corridor
PCD_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rpn_fastlio/20260416_175736.pcd"
YAML_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rpn_fastlio/20260416_175736.yaml"
NATURE="indoor"

# Non-secure CycloneDDS environment setup string (explicitly unsetting security)
NONSEC_ENV="source /opt/ros/humble/setup.bash && source /home/container_user/wheelchair2/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && unset ROS_SECURITY_ENABLE ROS_SECURITY_STRATEGY ROS_SECURITY_KEYSTORE ROS_SECURITY_ENCLAVE && export ROS_SECURITY_ENABLE=false && export ROS_DOMAIN_ID=56 && export CYCLONEDDS_URI=file:///home/container_user/wheelchair2/src/scripts/cyclonedds_jetson.xml"

# Parse silica_ros_f and video_web_server_f flags from backend_config.json
CONFIG_FILE="/home/ducky/wheelchair_ws/wheelchair_intelligence/development/config/backend_config.json"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/home/container_user/wheelchair_ws/google_quant/wheelchair_intelligence/development/config/backend_config.json"
fi
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/home/container_user/wheelchair2/src/dependencies/google_quant/wheelchair_intelligence/development/config/backend_config.json"
fi

if [ -f "$CONFIG_FILE" ]; then
    SILICA_ROS_F=$(python3 -c "import json; print(str(json.load(open('$CONFIG_FILE')).get('silica_ros_f', False)).lower())" 2>/dev/null || echo "false")
    VIDEO_WEB_SERVER_F=$(python3 -c "import json; print(str(json.load(open('$CONFIG_FILE')).get('video_web_server_f', False)).lower())" 2>/dev/null || echo "false")
else
    SILICA_ROS_F="false"
    VIDEO_WEB_SERVER_F="false"
fi


# Start detached session
tmux new-session -d -s $SESSION_NAME -n "bringup" -x 200 -y 100

################## 2. WINDOW 0: BRINGUP
# Pane 0.0: Livox
tmux send-keys -t $SESSION_NAME:0 "$NONSEC_ENV" C-m
tmux send-keys -t $SESSION_NAME:0 "ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True" C-m

# Pane 0.1: Controllers
tmux split-window -h -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0 "$NONSEC_ENV" C-m
tmux send-keys -t $SESSION_NAME:0 "sudo chmod 777 /dev/ttyACM0 && sleep 2 && ros2 launch wheelchair2_bringup wheelchair2.launch.py robot_name:=$WHEELCHAIR_NAME use_sim_time:=False" C-m

tmux select-layout -t $SESSION_NAME:0 tiled


################## 3. WINDOW 1: LOCALIZATION AND NAVIGATION
tmux new-window -t $SESSION_NAME -n "loc_nav"

# Pane 1.0: Localization
tmux send-keys -t $SESSION_NAME:1 "$NONSEC_ENV" C-m
tmux send-keys -t $SESSION_NAME:1 "ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=$NATURE | tee loc.log" C-m

# Pane 1.1: Navigation
tmux split-window -h -t $SESSION_NAME:1
tmux send-keys -t $SESSION_NAME:1 "$NONSEC_ENV" C-m
tmux send-keys -t $SESSION_NAME:1 "sleep 7 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$YAML_PATH nav2_rviz_required:=False | tee nav.log" C-m

tmux select-layout -t $SESSION_NAME:1 tiled


################## 4. WINDOW 2: SILICA GLASS DETECTION
tmux new-window -t $SESSION_NAME -n "silica"
tmux send-keys -t $SESSION_NAME:2 "$NONSEC_ENV" C-m
if [ "$SILICA_ROS_F" = "true" ]; then
    tmux send-keys -t $SESSION_NAME:2 "ros2 launch silica_ros align_metric.launch.py" C-m
else
    tmux send-keys -t $SESSION_NAME:2 "echo 'silica_ros_f is false (default). Skipping silica_ros launch.'" C-m
fi


################## 5. WINDOW 3: ROS2 BRIDGE & VIDEO SERVER
tmux new-window -t $SESSION_NAME -n "bridge"

# Pane 3.0: Bridge
tmux send-keys -t $SESSION_NAME:3 "$NONSEC_ENV" C-m
tmux send-keys -t $SESSION_NAME:3 "sleep 10 && ros2 run wheelchair_intelligence_ros2 e_intelligence_to_nav2_com" C-m

# Pane 3.1: Web Video Server
tmux split-window -h -t $SESSION_NAME:3
tmux send-keys -t $SESSION_NAME:3 "$NONSEC_ENV" C-m
if [ "$VIDEO_WEB_SERVER_F" = "true" ]; then
    tmux send-keys -t $SESSION_NAME:3 "sleep 5 && ros2 run web_video_server web_video_server" C-m
else
    tmux send-keys -t $SESSION_NAME:3 "echo 'video_web_server_f is false (default). Skipping web_video_server launch.'" C-m
fi

tmux select-layout -t $SESSION_NAME:3 tiled


################## ATTACH
# Start the user on the Bringup window
tmux select-window -t $SESSION_NAME:0
# tmux attach-session -t $SESSION_NAME
