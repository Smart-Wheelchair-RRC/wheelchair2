#!/bin/bash

# log commands and their arguments as they are executed
# exit on first failure
set -ex

# set up bash shortcuts by sourcing the shortcuts script in bashrc (safe check)
SHORTCUTS_LINE="source /home/container_user/wheelchair2/src/scripts/shortcuts.sh"
grep -qxF "$SHORTCUTS_LINE" /home/container_user/.bashrc || echo "$SHORTCUTS_LINE" >> /home/container_user/.bashrc

# source the local workspace (safe check)
SOURCE_LWS="source /home/container_user/wheelchair2/install/setup.bash"
grep -qxF "$SOURCE_LWS" /home/container_user/.bashrc || echo "$SOURCE_LWS" >> /home/container_user/.bashrc

# source ROS 2 Humble (safe check)
SOURCE_ROS="source /opt/ros/humble/setup.bash"
grep -qxF "$SOURCE_ROS" /home/container_user/.bashrc || echo "$SOURCE_ROS" >> /home/container_user/.bashrc

# --- Sourcing them now for the current script execution ---
source /opt/ros/humble/setup.bash
if [ -f "/home/container_user/wheelchair2/install/setup.bash" ]; then
    source /home/container_user/wheelchair2/install/setup.bash
fi

cd /home/container_user/wheelchair2

# Generate SROS2 keystore automatically on startup
if [ -f "/home/container_user/wheelchair2/install/setup.bash" ]; then
    ros2 run robot_security generate_keystore
else
    echo "Workspace install directory not found. Please compile the workspace first."
fi