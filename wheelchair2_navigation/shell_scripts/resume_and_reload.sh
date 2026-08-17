#!/bin/bash

# Use provided arguments, or default to the RRC Lab paths if none are provided
PCD_PATH=${1:-"/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/lab_map_with_outside_corridor.pcd"}
BASE_BBS_YAML=${2:-"/home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/config/ros2_test.yaml"}
NAV2_YAML_PATH=${3:-"/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/rrc/rrc_lab.yaml"}

echo "⚙️  PREPARATION: Updating Node Parameters..."

# Update ICP Localization (Now using the correct parameter: pcd_file_path)
if [ -n "$PCD_PATH" ]; then
    ros2 param set /icp_localization pcd_file_path "$PCD_PATH"
fi

# Dynamically Update 3D-BBS
if [ -n "$BASE_BBS_YAML" ] && [ -n "$PCD_PATH" ]; then
    TEMP_BBS_YAML="/tmp/current_floor_3dbbs.yaml"
    python3 -c "
import os, yaml
target_dir = os.path.dirname('$PCD_PATH')
with open('$BASE_BBS_YAML', 'r') as f:
    config_data = yaml.safe_load(f)
config_data['target_clouds'] = target_dir
with open('$TEMP_BBS_YAML', 'w') as f:
    yaml.safe_dump(config_data, f)
"
    ros2 param set /gpu_ros2_test_rviz2 config "$TEMP_BBS_YAML"
fi

echo "=========================================="
echo "▶️  STEP 1: Resuming DLIO Odometry"
echo "=========================================="
ros2 service call /pause_odom std_srvs/srv/SetBool "{data: false}"

echo "⏳ Waiting 3 seconds for DLIO to build initial live map..."
sleep 3

echo "=========================================="
echo "🎯 STEP 2: Reloading & Resuming 3D-BBS (Global Localization)"
echo "=========================================="
ros2 service call /load_3dbbs_config std_srvs/srv/Trigger "{}"

# Unpause so the node starts filling its LiDAR and IMU buffers again
ros2 service call /pause_3dbbs std_srvs/srv/SetBool "{data: false}"

echo "⏳ Waiting 3 seconds for LiDAR/IMU buffers to fill..."
sleep 3

echo "🔍 Triggering 3D-BBS Global Pose Estimation..."
# This simulates the RViz click to force localization on the new map
ros2 topic pub --once /click_loc std_msgs/msg/Bool "{data: true}"

echo "⏳ Waiting 3 seconds for 3D-BBS to calculate the pose..."
sleep 3

echo "=========================================="
echo "🗺️  STEP 3: Reloading & Resuming ICP and Map Server (Local Tracking)"
echo "=========================================="
if [ -n "$NAV2_YAML_PATH" ]; then
    ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: \"$NAV2_YAML_PATH\"}" > /dev/null
fi
ros2 service call /load_map std_srvs/srv/Trigger "{}"
ros2 service call /pause_localization std_srvs/srv/SetBool "{data: false}"

echo "✅ Floor transition sequence complete! Full system is online."