# RTABMap Wheelchair Implementation Documentation

## Overview

In this we implement the RTABMap using the RGBD realsense camera, 2D LiDAR and Wheel odometery.

### Dependencies 

- Make sure to build [Realsense SDK](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) 
###### Note that the included realsence ros wrapper is for kernal verion 6.8. If you are using < 6.8 please do delete the existing deps and follow the realsense official guide. CUDA version 12.6

- Install RTABMap using `sudo apt-get install ros-humble-rtabmap*`

- Teleop using joystick `sudo apt-get install ros-humble-teleop-twist-joy`


### Mapping 

- Run the following commands and telop the robot in the are which is to be mapped. 
- Run `sudo chmod 777 /dev/ttyACM0` if there is a port error with the controller
```
ros2 launch wheelchair2_bringup wheelchair2.launch.py in_simulation:=true
```
```
ros2 launch sllidar_ros2 sllidar_a3_launch.py 
```
```
ros2 launch realsense2_camera rs_launch.py
```
```
ros2 launch wheelchair2_control wheelchair_teleop.launch.py 
```
```
ros2 launch wheelchair2_navigation rtabmap_lidar_camera_icp.launch.py
```
- To view the created map run `rtabmap-databaseViewer ~/.ros/rtabmap.db`

### Localization
```
ros2 launch wheelchair2_navigation rtabmap_lidar_camera_icp.launch.py localization:=true
```