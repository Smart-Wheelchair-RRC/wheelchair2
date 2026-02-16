# Install Python dependencies for the Flutter App Bridge
pip install tornado pymongo

# Navigate to the workspace root
cd /home/container_user/wheelchair2

# Build the Rosbridge submodule and its core components
colcon build --symlink-install --packages-up-to rosbridge_server

# Automatically source the workspace for all future terminal sessions
if ! grep -q "source /home/container_user/wheelchair2/install/setup.bash" /home/container_user/.bashrc; then
  echo "source /home/container_user/wheelchair2/install/setup.bash" >> /home/container_user/.bashrc
fi