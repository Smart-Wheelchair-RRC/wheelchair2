#!/bin/bash
# Script to launch the 3D-BBS, ICP, and FastLIO localization and mapping stack

SESSION_NAME="3dbbs_icp_fastlio"

tmux has-session -t $SESSION_NAME &> /dev/null

if [ $? != 0 ] 
then
    # Create new session
    tmux new-session -d -s $SESSION_NAME

    # Pane 0: Wheelchair Bringup
    tmux send-keys -t $SESSION_NAME:0 "ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=False" C-m

    # Pane 1: Livox LiDAR
    tmux split-window -v -t $SESSION_NAME:0
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=False" C-m

    # # Pane 2: 3D-BBS
    # tmux split-window -h -t $SESSION_NAME:0.1
    # tmux send-keys -t $SESSION_NAME:0.2 "ros2 launch wheelchair2_3dbbs 3dbbs.launch.py" C-m

    # # Pane 3: FastLIO
    # tmux split-window -v -t $SESSION_NAME:0.2
    # tmux send-keys -t $SESSION_NAME:0.3 "ros2 launch fast_lio mapping.launch.py rviz:=false" C-m

    # # Wait for 5 seconds
    # tmux send-keys -t $SESSION_NAME:0.3 "sleep 5" C-m

    # # Pane 4: ICP Localization
    # tmux split-window -h -t $SESSION_NAME:0.0
    # tmux send-keys -t $SESSION_NAME:0.4 "ros2 launch icp_localization_ros2 bringup.launch.py rate:=10.0" C-m

    # Pane 5: Click Loc 1
    tmux split-window -v -t $SESSION_NAME:0.4
    tmux send-keys -t $SESSION_NAME:0.5 "python3 /home/container_user/wheelchair2/src/RRC-3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py" C-m

    # Pane 6: Click Loc 2
    tmux split-window -v -t $SESSION_NAME:0.5
    tmux send-keys -t $SESSION_NAME:0.6 "python3 /home/container_user/wheelchair2/src/wheelchair2_bringup/scripts/invert_lidar.py" C-m

fi

tmux attach-session -t $SESSION_NAME
