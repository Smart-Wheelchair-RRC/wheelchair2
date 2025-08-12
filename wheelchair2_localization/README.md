# Wheelchair2 Localization

This package is mean to be an all-encompassing solution for the localization/SLAM of the Wheelchair robot. (Except for the [AMCL of Nav2](https://index.ros.org/p/nav2_amcl/)) any localization method that is used in the Wheelchair2 robot should be implemented here.

## Current implementation
The current implementation uses 3D localization using the Iterative Closest Point (ICP) algorithm. This means that a pre-defined 3D map (referred to as "PCD map" or "3D map") should be provided as a file.

The current ICP algorithm is taken from [baiyeweiguang/icp_localization_ros2](https://github.com/baiyeweiguang/icp_localization_ros2), as it is a more-or-less simple but complete package and works with minimal modifications to the standard Nav2 stack.

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

    If you're in the wheelchair2 workspace, these should already be installed. If not, you can install Open3D using pip:

    ```bash
    python3 -m pip install open3d
    ```


## Usage
To use the localization package, simply run the launch file:

```bash
ros2 launch wheelchair2_localization localization_icp.launch.py
```

## Available nodes

| Launch sequence | Name | Package | Description |
| --- | --- | --- | --- |
| 1 | vizualize_pcd_map | [SELF] | An optional node that visualizes the 3D map in RViz using Open3D. This is useful for debugging and visualization purposes. |
| 2 | invert_livox_data_node | [SELF]  | Rotates the lidar scan 180 degrees around the x-axis, as our lidar is mounted upside-down. |
| 3 | icp_localization | icp_localization_ros2 | The main node that performs the ICP localization using the provided 3D map. |

## Parameters

Parameters can be configured EITHER in the [`localization_icp.launch.py`](launch/localization_icp.launch.py) file to store settings for the entire package OR provided as CLI arguments at run-time:

The priority of the parameters is as follows:
1.  Parameters from the command line when launching the package.
1.  Defaults from the Python launch file.

### Core functionality parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `lidar_inverted` | `bool` | `true` | Whether the lidar data is inverted (i.e., the lidar is mounted upside-down). |
| `use_sim_time` | `bool` | `false` | Whether to use simulated time (useful for testing in simulation environments). |
| `publish_pcd_map` | `bool` | `false` | Whether to publish the 3D map as a PCD file for visualization purposes. This is useful for debugging and visualization purposes, but not required for the localization to work. |

### Topics parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `input_lidar_topic` | `str` | `/livox/lidar` | | Raw lidar data is read, `invert_livox_data_node` node subscribes to this. |
| `inverted_lidar_topic` | `str` | `/livox/lidar_inverted` | Inverted lidar data is published. |
| `imu_topic` | `str` | `/livox/imu` | Raw IMU data is read, `invert_livox_data_node` node subscribes to this. |
| `inverted_imu_topic` | `str` | `/livox/imu_inverted` | Inverted IMU data is published. |
| `final_lidar_topic` | `str` | `/livox/lidar_inverted` | The `icp_localization` node subscribes to this. |
| `final_imu_topic` | `str` | `/livox/imu_inverted` | The `icp_localization` node subscribes to this, if IMU data is used. |
| `input_odom_topic` | `str` | `/odom` | The `icp_localization` node subscribes to this, if odometry data is used. |

### File paths parameters
| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `pcd_map_path` | `str` | `$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd` | The path to the 3D map file in PCD format, the `icp_localization` node uses this for localization. |
| `icp_config_file` | `str` | `dependencies/icp_localization_ros2/config/icp.yaml` | ICP algorithm params, taken from the `icp_localization_ros2` package. |
| `icp_node_params_file` | `str` | `dependencies/icp_localization_ros2/config/node_params.yaml` | ICP node parameters, taken from the `icp_localization_ros2` package. |