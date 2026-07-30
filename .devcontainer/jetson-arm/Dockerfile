FROM ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:dev5.2.1

# Add the following labels
LABEL org.opencontainers.image.description="Wheelchair2"
LABEL org.opencontainers.image.title="Wheelchair2Navigation"
LABEL org.opencontainers.image.vendor="Tarun R"
LABEL org.opencontainers.image.source="https://github.com/Smart-Wheelchair-RRC/wheelchair2.git"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# 1. Install required packages as root
USER root
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    libserial-dev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# 2. Switch to container_user
USER container_user

# 3. Copy files with ownership set to container_user to prevent permission errors
COPY --chown=container_user:container_user . /home/container_user/wheelchair2/src

WORKDIR /home/container_user/wheelchair2

# 4. Make shell scripts executable
RUN chmod +x src/wheelchair2_navigation/shell_scripts/*.tmux src/wheelchair2_navigation/shell_scripts/*.sh 2>/dev/null || true
