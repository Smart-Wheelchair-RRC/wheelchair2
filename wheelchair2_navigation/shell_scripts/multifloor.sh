#!/bin/bash

# Configuration
SESSION_NAME="wheelchair_navigation"
YAML_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/4th_floor_apr2026/2026_04_06.yaml"
PCD_PATH="/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/4th_floor_apr2026/2026_04_06.pcd"

# Log File
LOG_FILE="floor_transition_$(date +%Y%m%d_%H%M%S).log"

# --- COORDINATES (Your exact 4th Floor Elevator Exit) ---
INIT_X=-20.24
INIT_Y=-2.96
# Yaw -0.04 converted to Quaternion
INIT_Z=-0.01999
INIT_W=0.99979

log_info() {
    echo -e "[$(date +'%H:%M:%S')] [INFO] $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "[$(date +'%H:%M:%S')] [ERROR] $1" | tee -a "$LOG_FILE"
}

echo "------------------------------------------------" | tee -a "$LOG_FILE"
echo "🚀 STARTING HANDOVER: GROUND -> 4TH FLOOR" | tee -a "$LOG_FILE"
echo "------------------------------------------------" | tee -a "$LOG_FILE"

# ========================================================
# STAGE 1: MAP UPDATE (RUN IMMEDIATELY WHEN ENTERING LIFT)
# ========================================================
log_info "Updating Nav2 2D Map..."
SERVICE_RESULT=$(ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$YAML_PATH'}" 2>&1)

if [[ $? -eq 0 ]]; then
    log_info "Nav2 Map Server successfully updated."
    
    # CRITICAL: Force clear costmaps so the 'very big' map scales correctly in RViz
    log_info "Clearing global and local costmaps to force bounding box resize..."
    ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
    ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
else
    log_error "Failed to update Nav2 Map Server! Response: $SERVICE_RESULT"
fi

log_info "Restarting 3D Localization Stack with new PCD..."
# Sending C-c to Window 1, Pane 0 (Localization)
tmux send-keys -t $SESSION_NAME:1.0 C-c
sleep 1.5

# Sending the new launch command to start processing WHILE the lift moves
LAUNCH_CMD="ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=indoor"
tmux send-keys -t $SESSION_NAME:1.0 "$LAUNCH_CMD" C-m

if [[ $? -eq 0 ]]; then
    log_info "Launch command sent to tmux pane 1.0 successfully."
else
    log_error "Failed to send command to tmux!"
fi

echo -e "\n⏳ Maps are swapping in the background. Enjoy the lift ride up to the 4th floor..." | tee -a "$LOG_FILE"

# ========================================================
# STAGE 2: INITIALIZATION (AFTER EXITING LIFT)
# ========================================================
while true; do
    echo "------------------------------------------------"
    read -p "❓ Have you reached the 4th floor and EXITED the lift? (y/n): " yn
    case $yn in
        [Yy]* ) 
            log_info "User confirmed arrival on 4th floor. Initializing pose..."
            
            # Send the initial pose down the pipeline
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
                log_info "Coordinates: X:$INIT_X, Y:$INIT_Y, Yaw:-0.04"
                
                # CRITICAL STEP: Trigger the 3D-BBS node's click callback instantly 
                # to run global calculation right on top of our seed pose
                log_info "Triggering 3D-BBS matcher via /click_loc..."
                ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1
            else
                log_error "Failed to publish initial pose!"
                log_error "Result: $PUB_RESULT"
            fi
            
            echo -e "\n🎉 Handover Complete. Log saved to $LOG_FILE" | tee -a "$LOG_FILE"
            break;;
        * ) 
            log_info "Waiting for floor exit...";;
    esac
done