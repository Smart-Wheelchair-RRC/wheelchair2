#!/bin/bash
set -e

echo "[Wheelchair Startup] Waiting for /dev/ttyACM0..."
while [ ! -e /dev/ttyACM0 ]; do
    sleep 1
done
echo "[Wheelchair Startup] Device /dev/ttyACM0 found."

if docker ps -a --format '{{.Names}}' | grep -q '^mvp_test$'; then
    echo "[Wheelchair Startup] Starting existing container mvp_test"
    docker start -ai mvp_test
else
echo "[Wheelchair Startup] Creating and running new container mvp_test"
# Run container (no -t, so systemd won't fail)
docker run -t  \
  --name mvp_test \
  --runtime=nvidia \
  --cap-add=SYS_NICE \
  --ulimit rtprio=99 \
  --ulimit rttime=-1 \
  --ulimit memlock=8428281856 \
  --cap-add=all \
  --security-opt seccomp=unconfined \
  --security-opt apparmor=unconfined \
  --device=/dev/ttyACM0 \
  --volume=/dev:/dev \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --net=host \
  --ipc=host \
  -e DISPLAY=:1 \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e PULSE_SERVER=$PULSE_SERVER \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=5 \
  wheelchair-boot
fi
