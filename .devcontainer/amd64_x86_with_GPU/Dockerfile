FROM ghcr.io/anirudhadg/wheelchair2_base_gazebo:v4.0
# Add the following labels
LABEL org.opencontainers.image.description="Wheelchair2"
LABEL org.opencontainers.image.title="Wheelchair2Navigation"
LABEL org.opencontainers.image.vendor="Tarun R"
LABEL org.opencontainers.image.source="https://github.com/Smart-Wheelchair-RRC/wheelchair2.git"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"


USER root


# handle default shell
SHELL ["/bin/bash", "-c"]
COPY . /home/container_user/wheelchair2/src

WORKDIR /home/container_user/wheelchair2
RUN source /opt/ros/humble/setup.bash \
    && sudo colcon build --symlink-install 
    
WORKDIR /home/container_user/wheelchair2
RUN source install/setup.bash 

WORKDIR /home/container_user
RUN git clone https://github.com/koide3/small_gicp.git
RUN cd small_gicp \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j \
    && sudo make install \
    && cd /home/container_user \
    && rm -rf small_gicp


# CUDA
RUN sudo apt-get update && apt-get install -y --no-install-recommends \
gnupg2 \
ca-certificates \
&& sudo rm -rf /var/lib/apt/lists/*

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && sudo dpkg -i cuda-keyring_1.1-1_all.deb \
    && sudo rm cuda-keyring_1.1-1_all.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-toolkit-12-2 \
    && sudo rm -rf /var/lib/apt/lists/*


ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
