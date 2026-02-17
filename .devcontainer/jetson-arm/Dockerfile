FROM ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:dev4.3.7

# Add Metadata Labels
LABEL org.opencontainers.image.description="Wheelchair2"
LABEL org.opencontainers.image.title="Wheelchair2Navigation"
LABEL org.opencontainers.image.vendor="Tarun R"
LABEL org.opencontainers.image.source="https://github.com/Smart-Wheelchair-RRC/wheelchair2.git"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# Handle default shell
SHELL ["/bin/bash", "-c"]

USER root


RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake-mypy \
    ros-humble-rosbridge-server \
    ros-humble-rosapi \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-filters \
    ros-humble-laser-filters \
    ros-humble-diagnostic-updater \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install tornado pymongo

RUN mkdir -p /home/container_user/wheelchair2 && \
    chown -R container_user:container_user /home/container_user/wheelchair2

USER container_user
WORKDIR /home/container_user/wheelchair2