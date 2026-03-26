#!/bin/bash

SESSION_NAME="wheelchair_navigation"

# 1. Create a new tmux session in detached mode (This creates Pane 0)
tmux new-session -d -s $SESSION_NAME

# 2. Pane 0: Share click_loc.py (background) and box_filter.py (foreground)
tmux send-keys -t $SESSION_NAME "python3 /home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py &" C-m
# Short 1-second delay to ensure click_loc.py executes strictly first
tmux send-keys -t $SESSION_NAME "sleep 1" C-m
tmux send-keys -t $SESSION_NAME "python3 /home/container_user/wheelchair2/src/wheelchair2_bringup/scripts/box_filter.py" C-m

# 3. Pane 1: livox.launch.py
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True" C-m

# 4. Pane 2: wheelchair2.launch.py
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "ros2 launch wheelchair2_bringup wheelchair2.launch.py" C-m

# 5. Pane 3: sleep 7s then nav2_3d.launch.py
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 7 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/hardware_lab_2/hardware_lab_2.yaml | tee output.log" C-m

# 6. Arrange the 4 panes in a tiled layout
tmux select-layout -t $SESSION_NAME tiled

# 7. Attach to the session to view the output
tmux attach-session -t $SESSION_NAME