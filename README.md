# Wheelchair2
ROS2 Humble and Gazebo Harmonic packages to operate the RRC wheelchair

## Prerequisites

- **Ubuntu** (tested on 22.04)
- **VSCode**
- **Remote Development Extension by Microsoft** (Inside VSCode)
- **Docker Installation**
  ```bash
  # Install Docker using convenience script
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh ./get-docker.sh

  # Post-install configuration
  sudo groupadd docker
  sudo usermod -aG docker $USER

  # Verify if Docker service is enabled
  sudo systemctl is-enabled docker

  # If not enable it
  sudo systemctl enable docker.service
  sudo systemctl enable containerd.service
  ```
>[!IMPORTANT]
>**Reboot before proceeding further**
```bash
  # GHCR Authentication
  echo "<YOUR_GITHUB_PAT>" | docker login ghcr.io -u <YOUR_GITHUB_USERID> --password-stdin
  ```

* **Enabling Nvidia GPU for simulation**

  | Hardware | Requirement  |
  | :------- | :----------- |
  | GPU      | CUDA-enabled |

  | Software      | Requirement                                                           |
  | :------------ | :-------------------------------------------------------------------- |
  | Nvidia Driver | - Ubuntu 22.04 `>=515.43.04` <br/> - Ubuntu 24.04 `>=555.42.02` <br/> |

  - Switching to NVIDIA graphics driver libraries with prime-select tool (reboot required). By default, it is set to on-demand in which the system selects libraries to render automatically (not always correctly). 
  - So, run
  `sudo prime-select nvidia` locally.

## Hardware Setup
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

## Development Setup
- Create a folder for wheelchair development (preferably in home) 
  ```bash 
  mkdir wheelchair2 && cd wheelchair2
  # Clone the repo with submodules
  git clone https://github.com/Smart-Wheelchair-RRC/wheelchair2.git --recurse-submodules .
  # Open VSCode 
  code .
  ```
---
## How to Use
- **Set ROS DOMAIN ID** to a random number between 0 to 101 inside the **devcontainer.json** file. Please make sure to read the [Docs.](https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html#the-ros-domain-id-variable)
```json
{
  "containerEnv": {
    "ROS_DOMAIN_ID": "42"  // Change this to your unique ID between (0 - 101)
  }
}
```
- **Enter the container**
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select **Dev Containers: Rebuild and Reopen in Container**
    - Choose your development container:

      | Develeopment Container | System requirement         | Use case                        |
      |------------------------|----------------------------|---------------------------------|
      | Jetson                 | ARM system                 | Running Jetson                  |
      | AMD64/x86 with GPU     | x86 system with Nvidia GPU | Running Real-life + Simulation  |
      | AMD64/x86 without GPU  | x86 system                 | Running only Real-life          |

- **Build workspace and source**
  ```bash
  colcon build --symlink-install && source install/setup.bash
  ```
- **Wheelchair launch**
  ```bash
  # For real wheelchair
  sudo chmod 777 /dev/ttyACM0 # Permission for Motor Driver Controller
  ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=False use_joy:=True
  # For simulation
  ros2 launch wheelchair2_gazebo wheelchair2_sim.launch.py use_sim_time:=True use_joy:=True
  ```
- **LiDAR launch**
  ```bash
  ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=False 
  ```
- **Camera** 
  ```bash
  # Currently only used for rtabmap
  ros2 launch realsense2_camera rs_launch.py 
  ```
- **Mapping**
  ```bash
  # slam_toolbox
  ros2 launch wheelchair2_navigation online_async_launch.py
  # rtabmap 
  ros2 launch wheelchair2_navigation rtabmap_lidar_camera_icp.launch.py
  ```
- **Navigation** 
  ```bash
  ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=False map:=$MAP_DIR/rrc/rrc_lab.yaml
  ```
- **Keyboard Telep**
  ```bash
  ros2 run wheelchair2_control keyboard_teleop_constant.py
  ```
- **Data collection**
  ```bash
  ros2 launch wheelchair2_bringup data_collection.launch.py use_sim_time:=False use_camera1:=True use_camera2:=True use_rivz:=True record:=True
  ```
    
---

## Launch File Arguments

> [!NOTE]
>  The arguments shown above for each launch file are the default values.

| Argument      | Description                                                                         |
|---------------|-------------------------------------------------------------------------------------|
| `use_sim_time`| Launch in simulation mode (`True` for simulation, `False` for hardware connection). |
| `use_joy`     | Use joystick control.                                                               |
| `map`         | Specify map file for navigation.                                                    |
| `use_camera1` | Enable/disable first camera for data collection.                                    |
| `use_camera2` | Enable/disable second camera for data collection.                                   |
| `use_rivz`    | Enable/disable RViz visualization for data collection.                              |
| `record`      | Enable/disable data recording during data collection.                               |

---

## tmux Scripts

| Script File           | Action                                |
|-----------------------|---------------------------------------|
| `navigation.tmux`     | Runs real wheelchair navigation       |
| `sim_navigation.tmux` | Runs simulation wheelchair navigation |

Scripts are located in `wheelchair2_navigation/shell_scripts` for convenience.

---

## Demo Runs

>[!WARNING]
>This option is not maintained frequently so, the latest changes may not be applied here. 

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

> [!WARNING]
>  Recommended **not to run locally** — please use **Docker** instead.  
> Running locally may lead to **dependency conflicts** and could affect your **system-wide packages** or break existing setups.


