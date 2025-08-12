FROM ghcr.io/smart-wheelchair-rrc/wheelchair2_base_jetson:v4.1

# Use bash shell for consistent environment setup
SHELL ["/bin/bash", "-c"]

# Temporarily become root to fix file permissions
USER root
RUN chmod +x /ros_entrypoint.sh

# Switch back to non-root user expected by base image
USER container_user

# Add metadata labels
LABEL org.opencontainers.image.description="Wheelchair2"
LABEL org.opencontainers.image.title="Wheelchair2Navigation"
LABEL org.opencontainers.image.vendor="Tarun R"
LABEL org.opencontainers.image.source="https://github.com/Smart-Wheelchair-RRC/wheelchair2.git"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# Copy your source code
COPY . /home/container_user/wheelchair2/src

# Set working directory
WORKDIR /home/container_user/wheelchair2

# Temporarily switch to root
USER root

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && source install/setup.bash

RUN chown -R container_user:container_user /home/container_user/wheelchair2 \
    && chmod +x /home/container_user/wheelchair2/src/entrypoint.sh

# Switch back to non-root
USER container_user

ENTRYPOINT ["/home/container_user/wheelchair2/src/entrypoint.sh"]


