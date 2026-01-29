FROM ghcr.io/smart-wheelchair-rrc/wheelchair2_base_cuda:v5.0
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

# WORKDIR /home/container_user/wheelchair2
# RUN source /opt/ros/humble/setup.bash \
#     && sudo colcon build --symlink-install 


#Eigen3
WORKDIR /home/container_user
RUN sudo apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && sudo apt-get -y install --no-install-recommends libeigen3-dev

# 3D-BBS
WORKDIR /home/container_user
RUN git clone https://github.com/KOKIAOKI/3d_bbs.git --recursive --depth 1
RUN cd 3d_bbs \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j \
    && sudo make install \
    && cd /home/container_user \
    && rm -rf 3d_bbs \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
# WORKDIR /home/container_user/wheelchair2
# RUN source install/setup.bash 
# # && sudo chmod +x /home/container_user/wheelchair2/src/wheelchair2_navigation/shell_scripts/navigation.tmux
