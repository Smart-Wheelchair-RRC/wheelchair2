#!/bin/bash

echo "⏸️  1. Pausing ICP, DLIO, and 3D-BBS..."
ros2 service call /pause_localization std_srvs/srv/SetBool "{data: true}"
ros2 service call /pause_odom std_srvs/srv/SetBool "{data: true}"
ros2 service call /pause_3dbbs std_srvs/srv/SetBool "{data: true}"

echo "🗑️  2. Resetting maps, buffers, and accumulators..."
ros2 service call /reset_localization std_srvs/srv/Empty "{}"
ros2 service call /reset_map std_srvs/srv/Empty "{}"
ros2 service call /reset_3dbbs std_srvs/srv/Empty "{}"

echo "🧹 3. Clearing Nav2 Local Costmap (Ghost obstacles)..."
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"

echo "✅ Robot is fully paused and memory is wiped! Ready for floor transition."