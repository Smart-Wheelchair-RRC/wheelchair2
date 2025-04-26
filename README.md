# Wheelchair2
ROS2 Humble setup to operate the RRC wheelchair

## Table of contents

- [Wheelchair2](#wheelchair2)
  - [Table of contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
  - [Demo Runs](#demo-runs)
        - [x86 Laptop Navigation Demo run](#x86-laptop-navigation-demo-run)
        - [Jetson Navigation Demo run](#jetson-navigation-demo-run)

## Prerequisites

- Ubuntu 22.04 or 24.04
- VSCode
- Remote Development Extension by Microsoft (Inside VSCode)
- **Docker Installation**
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
  - **Reboot before proceeding further**
  ```bash
    # GHCR Authentication
    echo "<YOUR_GITHUB_PAT>" | docker login ghcr.io -u <YOUR_GITHUB_USERID> --password-stdin
    ```
- Enabling Nvidia GPU for simulation

  | Hardware | Requirement  |
  | :------- | :----------- |
  | GPU      | CUDA-enabled |

  | Software      | Requirement                                                           |
  | :------------ | :-------------------------------------------------------------------- |
  | Nvidia Driver | - Ubuntu 22.04 `>=515.43.04` <br/> - Ubuntu 24.04 `>=555.42.02` <br/> |
  
  - Switching to NVIDIA graphics driver libraries with prime-select tool (reboot required). By default, it is set to on-demand in which the system selects libraries to render automatically (not always correctly). 
  - So, run
  `sudo prime-select nvidia` locally.

## Hardware
Information about the Hardware Setup:

#### Motor Driver Controller and Wheel Odometery
  - **Components:**
    - Kangaroo X2 Motion controller
    - Sabertooth 2x32 Motor driver
    - AMT102-V Wheelencoders
#### RGB Camera: Realsense D455 
  - With the current setup we only use realsense for RTABMAP
  - No Additional Setup is required for getting data, just plug-in the type-c cable
#### 3D LiDAR: Livox MID-360
   1.  Create new network profile **"Livox"**
   2.  Configure static IP:
        - **IP Address:** 192.168.1.50
        - **Netmask:** 255.255.255.0

## Development 

#### Setup Process
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
    - Select **Dev Containers: Rebuild and Reopen in Container**
    - Choose your development container:
        - **Jetson** (ARM-based systems)

        - **AMD64/x86 with GPU** (Standard laptops)
        - **AMD64/x86 without GPU** (Machine with Nvidia GPU)
    - Use `Build WS` button to build workspace **or** run `colcon build --symlink-install && source install/setup.bash`
  
    >Note: To access these buttons you may need to enable [VSCode Action Button Extension](https://marketplace.visualstudio.com/items?itemName=seunlanlege.action-buttons) through the Extensions Tab in VSCode, the extension should download automatically on container startup

### How to Use

- ##### Wheelchair launch
  ```bash
  # For real wheelchair
  sudo chmod 777 /dev/ttyACM0 # Permission for Motor Driver Controller
  ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=<Boolean> use_joy:=<Boolean>
  # For simulation
  ros2 launch wheelchair2_gazebo wheelchair2_sim.launch.py use_sim_time:=<Boolean> use_joy:=<Boolean>
  ```
- ##### LiDAR launch
  ```bash
  ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=<Boolean> 
  ```
- ##### Camera 
  ```bash
  # Currently only used for rtabmap
  ros2 launch realsense2_camera rs_launch.py
  ```
- ##### Mapping 
  ```bash
  # slam_toolbox
  ros2 launch wheelchair2_navigation online_async_launch.py
  # rtabmap 
  ros2 launch wheelchair2_navigation rtabmap_lidar_camera_icp.launch.py
  ```
- ##### Navigation 
  ```bash
  ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=<Boolean> map:=$MAP_DIR/<map_folder>/<map_name>.yaml
  ```
- ##### Keyboard Telep
  ```bash
  ros2 run wheelchair2_control keyboard_teleop_constant.py
  ```

  Available arguments are as follows:

  | Argument       | Default Value               | Description                                                                         |
  | -------------- | --------------------------- | ----------------------------------------------------------------------------------- |
  | `use_sim_time` | `False`                     | Launch in simulation mode (`True` for simulation, `False` for hardware connection). |  |
  | `use_joy`      | `True`                      | Use joystick control.                                                               |
  | `map`          | `$MAP_DIR/rrc/rrc_lab.yaml` | Using different maps on startup.                                                    |

- Inside `wheelchair2_navigation/launch` the script files can be used for quick navigation and mapping. 

## Demo Runs

**⚠️ Important Note:** This option is not maintained frequently so, the latest changes may not be applied here. 

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


