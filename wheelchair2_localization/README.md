# Wheelchair2 Localization

This package is mean to be an all-encompassing solution for the localization/SLAM of the Wheelchair robot. (Except for the [AMCL of Nav2](https://index.ros.org/p/nav2_amcl/)) any localization method that is used in the Wheelchair2 robot should be implemented here.

## Current implementation
The current implementation uses 3D localization using the Iterative Closest Point (ICP) algorithm. This means that a pre-defined 3D map (referred to as "PCD map" or "3D map") should be provided as a file.

The current ICP algorithm is taken from [baiyeweiguang/icp_localization_ros2](https://github.com/baiyeweiguang/icp_localization_ros2), as it is a more-or-less simple but complete package and works with minimal modifications to the standard Nav2/AMCL stack.

## System requirements

1.  **`icp_localization_ros2`**: In the wheelchair2 workspace, this should most probably be pre-installed as a Git submodule. So you need only update/initialize the submodules.

    ```bash
    git submodule update --init --recursive --remote
    ```

    If not, simply add the submodule to the workspace:

    ```bash
    git submodule add https://github.com/baiyeweiguang/icp_localization_ros2.git dependencies/icp_localization_ros2
    ```

    Then, build the workspace:

    ```bash
    cd /home/container_user/wheelchair2 && colcon build --symlink-install
    ```

1.  **ROS2 packages**: These are simply the dependencies of the `icp_localization_ros2` package. Again, these should be pre-installed in the wheelchair2 workspace, but if not, you can install them using the following commands:

    ```bash
    sudo apt update && sudo apt install -y ros-humble-pcl-ros ros-humble-pcl-conversions libgoogle-glog-dev ros-humble-libpointmatcher
    ```

1.  **[OPTIONAL] Open3D dependency**: This package provides an extra node to visualize the 3D map in RViz. It is not required for the localization to work, but it is useful for debugging and visualization purposes, using Open3D.

    If you're in the wheelchair2 workspace, this should be available as a Python Virtual Environment, and may simply be activated if needed:
    
    ```bash
    source /home/container_user/python3_venvs/o3d_venv/bin/activate
    ```

    However, if this doesn't work, you can install Open3D using pip:

    ```bash
    python3 -m pip install open3d
    python3 -m pip install numpy==1.24
    ```


## Usage
To use the localization package, simply run the launch file:

```bash
ros2 launch wheelchair2_localization localization.launch.py
```

## Available nodes

| Launch sequence | Name | Package | Description |
| --- | --- | --- | --- |
| 1 | vizualize_pcd_map | [SELF] | An optional node that visualizes the 3D map in RViz using Open3D. This is useful for debugging and visualization purposes. |
| 2 | invert_livox_data_node | [SELF]  | Rotates the lidar scan 180 degrees around the x-axis, as our lidar is mounted upside-down. |
| 3 | icp_localization | icp_localization_ros2 | The main node that performs the ICP localization using the provided 3D map. |
| 4 | map_server | nav2_map_server | Provides the 2D map to the Nav2 stack. This is used for visualization purposes in RViz. |
| 5 | nav2_bringup/launch/navigation_launch.py launch file | nav2_bringup | The main Nav2 stack, except the map server and AMCL. This is used for navigation and path planning. |
| 6 | rviz2 | rviz2 | The RViz visualization tool, which is used to visualize the robot's state, the map, and the localization results. |

## Parameters

Parameters can be configured in the [`localization_config.yaml`](config/localization_config.yaml) file to store settings for the entire package:

This file allows for flexible configuration of the entire `wheelchair2_localization` package and all its components. The priority of the parameters is as follows:
1.  Defaults from the Python launch file.
2.  Parameters from the `localization_config.yaml` file.
3.  Parameters from the command line when launching the package.

### Core functionality parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `lidar_inverted` | `bool` | `true` | Whether the lidar data is inverted (i.e., the lidar is mounted upside-down). |
| `use_navigation` | `bool` | `true` | Whether to use the Nav2 stack for navigation and path planning. |
| `use_sim_time` | `bool` | `false` | Whether to use simulated time (useful for testing in simulation environments). |
| `nav2_rviz_required` | `bool` | `true` | Whether to launch the RViz visualization tool for the Nav2 stack. |
| `use_mapper_server` | `bool` | `true` | Whether to use the Nav2 map server to provide the 2D map to the Nav2 stack. |
| `publish_pcd_map` | `bool` | `false` | Whether to publish the 3D map as a PCD file for visualization purposes. This is useful for debugging and visualization purposes, but not required for the localization to work. |

### Frames and TFs parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `main_map_frame` | `str` | `map` | The main map frame used for localization. This is the frame in which both the 3D and 2D maps are provided. |
| `main_odom_frame` | `str` | `odom` | The main odometry frame used for localization. This is the frame in which the robot's odometry is provided. |

### Topics parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `input_lidar_topic` | `str` | `/livox/lidar` | | The topic from which the raw lidar data is read. This is the topic that the `invert_livox_data_node` node subscribes to. |
| `inverted_lidar_topic` | `str` | `/livox/lidar_inverted` | The topic to which the inverted lidar data is published. |
| `imu_topic` | `str` | `/livox/imu` | The topic from which the raw IMU data is read. This is the topic that the `invert_livox_data_node` node subscribes to. |
| `inverted_imu_topic` | `str` | `/livox/imu_inverted` | The topic to which the inverted IMU data is published. |
| `final_lidar_topic` | `str` | `/livox/lidar_inverted` | The topic to which the final inverted lidar data is published. This is the topic that the `icp_localization` node subscribes to. |
| `final_imu_topic` | `str` | `/livox/imu_inverted` | The topic to which the final inverted IMU data is published. This is the topic that the `icp_localization` node subscribes to, if IMU data is used. |
| `input_odom_topic` | `str` | `/odom` | The topic from which the raw odometry data is read. This is the topic that the `icp_localization` node subscribes to, if odometry data is used. |

### File paths parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `pcd_map_path` | `str` | `$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd` | The path to the 3D map file in PCD format. This is the file that the `icp_localization` node uses for localization. |
| `map_file_path` | `str` | `$MAP_DIR/rrc/rrc_lab.yaml` | The path to the 2D map file in YAML format. This is the file that the `nav2_map_server` node uses to provide the 2D map to the Nav2 stack. |
| `icp_config_file` | `str` | `dependencies/icp_localization_ros2/config/icp.yaml` | The path to the ICP configuration file. This file contains the parameters for the ICP algorithm, such as the maximum number of iterations, the convergence threshold, etc. |
| `icp_node_params_file` | `str` | `dependencies/icp_localization_ros2/config/node_params.yaml` | The path to the ICP node parameters file. This file contains the parameters for the `icp_localization` node, such as the input and output topics, the frame IDs, etc. |
| `nav2_params_file` | `str` | `wheelchair2_navigation/config/nav2_params.yaml` | The path to the Nav2 parameters file. This file contains the parameters for the Nav2 stack, such as the planner, controller, etc. |
| `nav2_rviz_config_file` | `str` | `wheelchair2_navigation/rviz/navigation.rviz` | The path to the RViz configuration file for the Nav2 stack. This file contains the configuration for the RViz visualization tool, such as the topics to visualize, the frames to use, etc. |    