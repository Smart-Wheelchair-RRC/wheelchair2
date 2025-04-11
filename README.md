# Wheelchair2
ROS2 Humble setup to operate the RRC wheelchair

## Table of contents

- [Wheelchair2](#wheelchair2)
  - [Table of contents](#table-of-contents)
    - [Docker Setup](#docker-setup)
      - [Installation \& Configuration](#installation--configuration)
      - [Docker Installation](#docker-installation)
        - [GHCR Authentication](#ghcr-authentication)
    - [Hardware Configuration](#hardware-configuration)
        - [Motor Driver Controller and Wheel Odometery](#motor-driver-controller-and-wheel-odometery)
        - [RGB Camera: Realsense D455](#rgb-camera-realsense-d455)
        - [3D LiDAR: Livox MID-360](#3d-lidar-livox-mid-360)
    - [Software Setup Options](#software-setup-options)
      - [Option 1: Docker for Development](#option-1-docker-for-development)
        - [Prerequisites](#prerequisites)
        - [Setup Process](#setup-process)
      - [Option 2: Docker for Demo runs](#option-2-docker-for-demo-runs)
        - [x86 Laptop Navigation Demo run](#x86-laptop-navigation-demo-run)
        - [Jetson Navigation Demo run](#jetson-navigation-demo-run)


### Docker Setup
#### Installation & Configuration
#### Docker Installation
  ```bash
  # Install Docker using convenience script
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh ./get-docker.sh

  # Post-install configuration
  sudo groupadd docker
  sudo usermod -aG docker $USER
  sudo systemctl enable docker.service
  sudo systemctl enable containerd.service

  # Verify installation
  sudo systemctl is-enabled docker
  ```

 **Reboot before proceeding further**

##### GHCR Authentication 
  ```bash
  echo "<YOUR_GITHUB_PAT>" | docker login ghcr.io -u <YOUR_GITHUB_USERID> --password-stdin
  ```
### Hardware Configuration
The repository has currently been setup with
##### Motor Driver Controller and Wheel Odometery
  - **Components:**
    - Kangaroo X2 Motion controller
    - Sabertooth 2x32 Motor driver
    - AMT102-V Wheelencoders

**Permission for Motor Driver Controller**
```bash 
sudo chmod 777 /dev/ttyACM0
```

##### RGB Camera: Realsense D455 
  - With the current setup we only use realsense for RTABMAP
  - No Additional Setup is required for getting data, just plug-in the type-c cable
##### 3D LiDAR: Livox MID-360
   1.  Create new network profile "Livox"
   2.  Configure static IP:
        - **IP Address:** 192.168.1.50
        - **Netmask:** 255.255.255.0

### Software Setup Options

#### Option 1: Docker for Development
##### Prerequisites
- VSCode
- Remote Development Extension by Microsoft (Inside VSCode)
  
##### Setup Process
- Create a folder for wheelchair development (preferably in home) 
    ```bash 
    mkdir wheelchair2 && cd wheelchair2
    # Clone the repo with submodules
    git clone https://github.com/Smart-Wheelchair-RRC/wheelchair2.git --recurse-submodules .
    # Open VSCode 
    code .
    ```
- To enter the container
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select **Dev Containers: Reopen in Container**
    - Choose architecture:
        - **Jetson** (ARM-based systems)

        - **AMD64/x86** (Standard laptops)
    >Note: If there is an error related to gpu, comment the line 53 in the devcontainer.json and try to build again
    - Use `Build WS` button to build workspace
  
    >Note: To access these buttons you may need to enable [VSCode Action Button Extension](https://marketplace.visualstudio.com/items?itemName=seunlanlege.action-buttons) through the Extensions Tab in VSCode, the extension should download automatically on container startup
  
- ##### To run the Navigation stack inside the container
  ```bash
  ros2 launch wheelchair2_bringup wheelchair2_navigation_rrc.launch.py use_keepout_zone:=<Boolean> use_joy:=<Boolean>
  ```
  - use_joy: To use joystick teleop (only works on AMD64x86)
  - use_keepout_zone: To use keepout zone.
  - Change `wheelchair2_navigation_rrc` to `wheelchair2_navigation_kcis` when working at KCIS block.
  - Keyboard teleop `ros2 run wheelchair2_control keyboard_teleop_constant.py`

#### Option 2: Docker for Demo runs

##### x86 Laptop Navigation Demo run
- To pull latest docker image
    ```bash
    docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_navigation:v1.0
    ```  
- To start container
    ```bash
    docker run -it \
    --rm \
    --privileged \
    --cap-add=SYS_NICE \
    --ulimit rtprio=99 \
    --ulimit rttime=-1 \
    --ulimit memlock=8428281856 \
    --cap-add=all \
    --security-opt seccomp:unconfined \
    --security-opt apparmor:unconfined \
    --volume=/dev:/dev \
    --net=host \
    --ipc=host \
    -e DISPLAY="${DISPLAY}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
    -e PULSE_SERVER="${PULSE_SERVER}" \
    -e QT_X11_NO_MITSHM="1" \
    -e LIBGL_ALWAYS_SOFTWARE="1" \
    --entrypoint /bin/bash \
    --name wheelchair2-navigation \
    ghcr.io/smart-wheelchair-rrc/wheelchair2_navigation:v1.0
    ```
- Inside the container to run the Navigation stack - follow the above [instructions](#to-run-the-navigation-stack-inside-the-container)

##### Jetson Navigation Demo run

- To pull latest docker image
    ```bash
    docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_navigation_jetson:v1.0
    ```  
- To start container
    ```bash
    docker run -it \
    --rm \
    --privileged \
    --cap-add=SYS_NICE \
    --ulimit rtprio=99 \
    --ulimit rttime=-1 \
    --ulimit memlock=8428281856 \
    --cap-add=all \
    --security-opt seccomp:unconfined \
    --security-opt apparmor:unconfined \
    --volume=/dev:/dev \
    --net=host \
    --ipc=host \
    -e DISPLAY="${DISPLAY}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
    -e PULSE_SERVER="${PULSE_SERVER}" \
    -e QT_X11_NO_MITSHM="1" \
    -e LIBGL_ALWAYS_SOFTWARE="1" \
    --entrypoint /bin/bash \
    --name wheelchair2-navigation-jetson \
    ghcr.io/smart-wheelchair-rrc/wheelchair2_navigation_jetson:v1.0
    ```
- Inside the container to run the Navigation stack - follow the above [instructions](#to-run-the-navigation-stack-inside-the-container)

> ⚠️ **Warning:** Recommended **not to run locally** — please use **Docker** instead.  
> Running locally may lead to **dependency conflicts** and could affect your **system-wide packages** or break existing setups.


