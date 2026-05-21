#!/bin/bash

# Configuration
SESSION_NAME="wheelchair_navigation"
LOG_FILE="floor_transition_$(date +%Y%m%d_%H%M%S).log"

log_info() {
    echo -e "[$(date +'%H:%M:%S')] [INFO] $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "[$(date +'%H:%M:%S')] [ERROR] $1" | tee -a "$LOG_FILE"
}

echo "------------------------------------------------" | tee -a "$LOG_FILE"
echo "🛗  ROBUST MULTI-FLOOR HANDOVER MANAGER (FAST MAP LOAD)" | tee -a "$LOG_FILE"
echo "------------------------------------------------" | tee -a "$LOG_FILE"

# ========================================================
# ASK WHICH FLOOR (1 OR 2)
# ========================================================
echo "Select target destination floor layout:"
echo "1) GROUND FLOOR (Hardware Lab 2)"
echo "2) 4TH FLOOR (RRC Autonomous Corridor Layout)"
read -p "Enter choice [1 or 2]: " floor_choice

if [ "$floor_choice" == "1" ]; then
    TARGET_NAME="GROUND FLOOR"
    YAML_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/hardware_lab_2.yaml"
    PCD_PATH="/home/container_user/wheelchair2/src/hardware_lab_2.pcd"
    INIT_X=0.0
    INIT_Y=0.0
    INIT_Z=0.0
    INIT_W=1.0
elif [ "$floor_choice" == "2" ]; then
    TARGET_NAME="4TH FLOOR"
    YAML_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/4th_floor_apr2026/2026_04_06.yaml"
    PCD_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/4th_floor_apr2026/2026_04_06.pcd"
    INIT_X=-20.24
    INIT_Y=-2.96
    INIT_Z=-0.01999
    INIT_W=0.99979
else
    log_error "Invalid selection index. Aborting process tracker."
    exit 1
fi

log_info "Target Approved: [$TARGET_NAME]"

# ========================================================
# STAGE 1: ASYNCHRONOUS MAP UPDATE (RUNS IN BACKGROUND)
# ========================================================
log_info "Launching Nav2 2D Map update in the background..."
# The '&' at the end makes this run in the background so the script moves on instantly!
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$YAML_PATH'}" > /dev/null 2>&1 &

log_info "Restarting 3D Localization Stack with new PCD..."
# Sending C-c to Window 1, Pane 0 (Localization)
tmux send-keys -t $SESSION_NAME:1.0 C-c
sleep 0.5 # Kept at minimum to prevent wasting time

# Sending the new launch command to start processing immediately
LAUNCH_CMD="ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=indoor"
tmux send-keys -t $SESSION_NAME:1.0 "$LAUNCH_CMD" C-m

if [[ $? -eq 0 ]]; then
    log_info "Launch command sent to tmux pane 1.0 successfully."
else
    log_error "Failed to send command to tmux!"
fi

echo -e "\n⏳ Operations running in parallel. Enjoy the lift ride up to the $TARGET_NAME..." | tee -a "$LOG_FILE"

# ========================================================
# STAGE 2: INITIALIZATION (BACKGROUND MONITOR WATCHDOG LOOP)
# ========================================================
echo "------------------------------------------------"
log_info "⌛ Stage 2: Background /score stream monitor active..."
log_info "Waiting for 3D-BBS node alignment calculation..."
echo "Please keep the wheelchair static inside or exiting the lift."
echo "------------------------------------------------"

start_time=$(date +%s)

FIFO_PIPE="/tmp/ros2_score_pipe_$$"
mkfifo "$FIFO_PIPE"

# Launch continuous echo thread in the background, dumping outputs into our pipe file
ros2 topic echo /score 2>/dev/null > "$FIFO_PIPE" &
BACKGROUND_PID=$!

# Ensure the background task is terminated safely if the user exits the script manually
trap "kill $BACKGROUND_PID 2>/dev/null; rm -f $FIFO_PIPE" EXIT

while true; do
    # Perform a non-blocking read operation with a 1-second timeout on the pipe
    if read -t 1 RAW_LINE < "$FIFO_PIPE"; then
        if [[ "$RAW_LINE" == *"data:"* ]]; then
            current_score=$(echo "$RAW_LINE" | tr -dc '0-9')
            if [ -n "$current_score" ] && [ "$current_score" -gt 0 ]; then
                echo "" # Clear layout display line carriage
                log_info "✅ [SCORE DETECTED] Match lock successfully captured! Value: $current_score"
                log_info "Initializing pose alignment vectors on $TARGET_NAME..."
                break
            fi
        fi
    fi

    current_time=$(date +%s)
    elapsed=$((current_time - start_time))
    echo -ne "\r[$(date +'%H:%M:%S')] [PROCESSING] Waiting for 3D-BBS cloud alignment... (${elapsed}s elapsed)"
done

# Clean up our background process handles immediately
kill $BACKGROUND_PID 2>/dev/null
rm -f $FIFO_PIPE

# ========================================================
# STAGE 3: FLUSH COSTMAPS & INJECT INITIAL POSE TRANSFORMS
# ========================================================
echo "------------------------------------------------"
log_info "Clearing costmaps to force bounding box resize..."
# Run costmap clears in the background asynchronously so we don't wait for them
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1 &
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1 &

log_info "Publishing initial pose estimation positioning coordinates..."
PUB_RESULT=$(ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
  pose: {
    pose: {
      position: {x: $INIT_X, y: $INIT_Y, z: 0.0},
      orientation: {z: $INIT_Z, w: $INIT_W}
    }
  }
}" 2>&1)

if [[ $? -eq 0 ]]; then
    log_info "Pose initialization message published successfully."
    log_info "Coordinates: X:$INIT_X, Y:$INIT_Y"
    
    # Trigger the 3D-BBS node's click callback instantly to force matrix lock calculation
    log_info "Triggering 3D-BBS matcher via /click_loc..."
    ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1
else
    log_error "Failed to publish initial pose!"
    log_error "Result: $PUB_RESULT"
fi

# Ensure all background service tasks wrap up completely before exiting
wait

echo -e "\n🎉 Handover Complete. Log saved to $LOG_FILE" | tee -a "$LOG_FILE"