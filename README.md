# Wheelchair2
ROS2 Humble setup to operate the RRC wheelchair

## Table of contents

- [Wheelchair2](#wheelchair2)
  - [Table of contents](#table-of-contents)
  - [Docker Setup](#docker-setup)
      - [Install and setup Docker](#install-and-setup-docker)
      - [Option 1: Docker for Development](#option-1-docker-for-development)
        - [Prerequisites](#prerequisites)
        - [Wheelchair Development](#wheelchair-development)
      - [Option 2: Docker for demo runs](#option-2-docker-for-demo-runs)
  - [Hardware Setup](#hardware-setup)
  - [How to Use Locally](#how-to-use-locally)


## Docker Setup
- The docker can be used in two different setting. 
  - Option 1: Docker for development 
  - Option 2: Docker for demo runs
>
#### Install and setup Docker
- Install using simple bash script
  ```bash
  curl -fsSL https://get.docker.com -o get-docker.sh
  sudo sh ./get-docker.sh
  ```
- Post-Install setup
  ```bash
  sudo groupadd docker
  sudo usermod -aG docker $USER
  ```
- Configure Docker to start on boot with systemd
  ```bash
  sudo systemctl enable docker.service
  sudo systemctl enable containerd.service
  ```
- Check if it is enabled 
  ```bash
  sudo systemctl is-enabled docker
  ```
- **Reboot before proceeding further**
- Login to GHCR using your GitHub  
  ```bash
  echo "<YOUR_GITHUB_PAT>" | docker login ghcr.io -u <YOUR_GITHUB_USERID> --password-stdin
  ```
#### Option 1: Docker for Development
##### Prerequisites
- VSCode
- Remote Development Extension by Microsoft
  
##### Wheelchair Development 
- Create a folder for wheelchair development (preferably in home) 
    ```bash 
    mkdir wheelchair2 && cd wheelchair2
    ```
- Clone the repo with submodules
    ```bash
    git clone https://github.com/Smart-Wheelchair-RRC/wheelchair2.git --recurse-submodules .
    ```
- To start container
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select Option to DevContainer: Rebuild and Reopen Container (the process takes some time during the first time)
      >Note: If there is an error related to gpu, comment the line 53 in the devcontainer.json and try to build again
    - Use `Build WS` button to build workspace
  
      >Note: To access these buttons you may need to enable [VSCode Action Button Extension](https://marketplace.visualstudio.com/items?itemName=seunlanlege.action-buttons) through the Extensions Tab in VSCode, the extension should download automatically on container startup
  
- To run the Navigation stack inside the container
  ```
  source install/setup.bash && cd src/wheelchair2_navigation/rviz && ./navigation.sh
  ```

#### Option 2: Docker for demo runs
- To pull latest docker image
    ```bash
    docker pull ghcr.io/smart-wheelchair-rrc/wheelchair2_navigation:v1.0.0
    ```  
- To start container
    ```bash
    docker run -it \
    --rm \
    --privileged \
    --gpus=all \
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
    ghcr.io/smart-wheelchair-rrc/wheelchair2_navigation:v1.0.0
    ```
- Inside the container to run the Navigation stack
    ```bash
    source install/setup.bash && cd src/wheelchair2_navigation/rviz && ./navigation.sh
    ```
  
## Hardware Setup
The repository has currently been setup with
- Wheel Odometery
  - Kangaroo X2 Motion controller
  - Sabertooth 2x32 Motor driver
  - AMT102-V Wheelencoders
- RGB Camera: Realsense D455 (for RTABMap only)
  - No Additional Setup is required for this
- 3D LiDAR: Livox MID-360
  - IP Address: 192.168.1.50
  - Netmask: 255.255.255.0

## How to Use Locally
- Clone the repo inside your workspace
  ```bash
  mkdir wheelchair2_ws && cd wheelchair2_ws && mkdir src && cd src
  git clone https://github.com/Smart-Wheelchair-RRC/wheelchair2.git --recurse-submodules
  ``` 
- Build and Source the Workspace using below commands (in `/wheelchair2_ws` folder)
  ```bash
  colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
  source install/setup.bash
  ```
- Ensure Hardware is setup properly connected with the laptop and powered on
- Use following command to give permission for the Kangaroo X2 controller
  ```bash
  sudo chmod 777 /dev/ttyACM0
  ```
- Use following command to launch the Kangaroo X2 controller, State publisher and Teleop node
  ```bash
  ros2 launch wheelchair2_bringup wheelchair2.launch.py in_simulation:=false
  ```
- Use following command to launch 3D LiDAR and pointcloud to laserscan node
  ```bash
  ros2 launch wheelchair2_bringup livox.launch.py
  ```
- Use following command to launch the navigation stack and Rviz
  ```bash
  ros2 launch wheelchair2_navigation nav2.launch.py use_sim_time:=False
  ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
  ```
  >Note: You can also navigate to the src/wheelchair2_navigation/rviz and run `rviz2 -d navigation_amcl.rviz` For visualizing with the camera PoV. 
- A few launch option are available with above command
  - `in_simulation` and `use_sim_time:=True/False` : False to run the hardware setup ,True to run Gazebo
