FROM ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v1.0

# Add the following labels
LABEL org.opencontainers.image.description="Wheelchair2"
LABEL org.opencontainers.image.title="Wheelchair2Navigation"
LABEL org.opencontainers.image.vendor="Tarun R"
LABEL org.opencontainers.image.source="https://github.com/Smart-Wheelchair-RRC/wheelchair2.git"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

COPY . /home/wheelchair2/wheelchair2/src

WORKDIR /home/wheelchair2/wheelchair2
# handle default shell
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble \
    && colcon build --packages-select wheelchair2_navigation wheelchair2_control wheelchair2_bringup --symlink-install \
    && source install/setup.bash \
    && cd src/wheelchair2_navigation/launch \
    && sudo chmod +x ./navigation.sh
