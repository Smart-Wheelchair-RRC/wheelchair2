FROM ghcr.io/smart-wheelchair-rrc/wheelchair2_base:v4.1

# Add the following labels
LABEL org.opencontainers.image.description="Wheelchair2"
LABEL org.opencontainers.image.title="Wheelchair2Navigation"
LABEL org.opencontainers.image.vendor="Tarun R"
LABEL org.opencontainers.image.source="https://github.com/Smart-Wheelchair-RRC/wheelchair2.git"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# handle default shell
SHELL ["/bin/bash", "-c"]
COPY . /home/container_user/wheelchair2/src

WORKDIR /home/container_user/wheelchair2

RUN source /opt/ros/humble/setup.bash \
    && sudo colcon build --symlink-install \
    && source install/setup.bash \
    && sudo chmod +x /home/container_user/wheelchair2/src/wheelchair2_navigation/shell_scripts/navigation.tmux

