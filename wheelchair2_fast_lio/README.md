# FAST-LIO package for Wheelchair2

This a ROS2 implementation of the FAST-LIO algorithm for the Wheelchair2 robot. Odometry information and relevant frames are published by the `fastlio_mapping` node. You should consult the docs of the `FAST_LIO_ROS2` package for more details on the base FAST-LIO functionality.

Localization is performed using our own `localization`, which uses a pre-built PCD map.

Original paper on FAST-LIO:

```bibtex
@ARTICLE{fast-lio-paper,
    author={Xu, Wei and Zhang, Fu},
    journal={IEEE Robotics and Automation Letters}, 
    title={FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter}, 
    year={2021},
    volume={6},
    number={2},
    pages={3317-3324},
    abstract={This letter presents a computationally efficient and robust LiDAR-inertial odometry framework. We fuse LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. To lower the computation load in the presence of a large number of measurements, we present a new formula to compute the Kalman gain. The new formula has computation load depending on the state dimension instead of the measurement dimension. The proposed method and its implementation are tested in various indoor and outdoor environments. In all tests, our method produces reliable navigation results in real-time: running on a quadrotor onboard computer, it fuses more than 1200 effective feature points in a scan and completes all iterations of an iEKF step within 25 ms. Our codes are open-sourced on Github.1},
    keywords={Laser radar;Kalman filters;Fuses;Feature extraction;Distortion measurement;Three-dimensional displays;Noise measurement;Aerial systems;localization;perception and autonomy;sensor fusion},
    doi={10.1109/LRA.2021.3064227},
    ISSN={2377-3766},
    month={April},
}
```

## Requirements and installation

1.  **FAST_LIO_ROS2**

    You must have the [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) package added to your project.

    The recommended way to do this is to add this to your repository dependencies as a git submodule (this is already done for you in the Wheelchair2 repository):

    ```bash
    git submodule add https://github.com/Ericsii/FAST_LIO_ROS2.git dependencies/FAST_LIO_ROS2
    ```

    Next, make sure the submodule is set to the `ros2` branch by editing the [`.gitmodules`](/.gitmodules) file in the root of your repository (this is also done for you in the Wheelchair2 repository):

    ```ini
    [submodule "dependencies/FAST_LIO_ROS2"]
        path = dependencies/FAST_LIO_ROS2
        url = https://github.com/Ericsii/FAST_LIO_ROS2.git
        branch = ros2
    ```

    Make sure to run the following command to initialize the submodule (**you must do this**):

    ```bash
    git submodule update --init --recursive --remote
    ```


2.  **Python PyPa packages**

    This package requires the following Python packages:
    1. numpy 1.24.x
    2. open3d

    **Important**: These must be installed in the specified order, otherwise open3d may install a different version of numpy:

    ```bash
    python3 -m pip install open3d
    python3 -m pip install numpy==1.24
    ```

3.  Finally, make sure to build the package:

    ```bash
    colcon build --symlink-install
    ```

## Available Nodes

| Node | Description |
| --- | --- |
| `invert_livox_data` | Inverts the lidar and IMU data from the `livox` driver. |
| `tf_fast_lio_init_frame` | Publishes the tf from `odom` to `camera_init`. |
| `localization` | Localization using a pre-built PCD map. |
| `fastlio_mapping` (from the `FAST_LIO_ROS2` package) | The main FAST-LIO node that performs the odometry. |
| `map_server` (from `nav2_map_server`) | Loads and serves the map for Nav2. Launched if `navigation_mode` is true. |
| Nav2 Stack (via `navigation_launch.py`) | Core Nav2 components (planner, controller, behavior server, etc.) for navigation. Launched if `navigation_mode` is true. |

## Usage

To run the package using the launch file, simple source the workspace and run the following command:

```bash
ros2 launch wheelchair2_fast_lio fast_lio.launch.py
```

### Sequence of launching nodes
1. `invert_livox_data`: This node inverts the lidar and IMU data from the `livox` driver. It is launched first to ensure that the data is inverted before being passed to the FAST-LIO node. By inversion we mean a 180-degree rotation about the x-axis, since the lidar is mounted upside down on the wheelchair.
2. `tf_fast_lio_init_frame`: This node publishes the tf from `odom` to `camera_init`. It is launched after the `invert_livox_data` node to ensure that the tf is published after the data is inverted.
3. `fastlio_mapping`: This is the main FAST-LIO node that performs the odometry. It is launched after the `invert_livox_data` and `tf_fast_lio_init_frame` nodes to ensure that it has access to the inverted data and the tf.
4. `localization`: This node provides localization using a pre-built PCD map. It is launched after the FAST-LIO node to make use of the odometry data.
5. Nav2 components (if `navigation_mode` is true):
    - `map_server`: Loads the 2D map required for navigation.
    - `lifecycle_manager` for `map_server`: Manages the lifecycle of the `map_server`.
    - `navigation_launch.py`: Launches the core Nav2 stack (planner, controller, behavior server, etc.).
6. `rviz2`: This node launches RViz with the specified config file(s). It is launched last to ensure that all the other nodes are running before launching RViz. Separate RViz instances can be launched for FAST-LIO and Nav2 visualization based on their respective `*_rviz_required` parameters.

### Parameters

The following parameters are available in the launch file:

| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| **Core functionality** |
| `use_sim_time` | `bool` | `false` | Whether to use simulation time |
| `lidar_inverted` | `bool` | `true` | Whether to invert the lidar and IMU data (180-degree rotation about the x-axis) |
| `use_localization` | `bool` | `true` | Whether to use localization mode |
| `navigation_mode` | `bool` | `true` | Whether to enable navigation mode with Nav2 |
| **Frame and TF configuration** |
| `odom_frame_id` | `str` | `odom` | Shared odometry frame ID |
| `initial_frame_id` | `str` | `camera_init` | Initial camera frame ID for `tf_fast_lio_init_frame` node |
| `loc_global_frame` | `str` | `map` | Global frame ID for localization node (e.g., map) |
| `init_frame_tf_required` | `bool` | `true` | Whether to publish tf from `odom` to `camera_init` |
| `loc_broadcast_tf` | `bool` | `true` | Whether localization node broadcasts map to odom TF |
| **Configuration files** |
| `fast_lio_config` | `str` (file name, not path) | `fast_lio_mapping_mid360.yaml` | Name of the `fast_lio` config file |
| `fast_lio_config_path` | `str` (folder path) | `wheelchair2_fast_lio/config` | Path to the `fast_lio` config file directory |
| `pcd_map_path` | `str` (file path) | `you-super-accurate-map.pcd` | Path to the PCD map file for localization |
| `nav2_params_file` | `str` (file path) | `wheelchair2_fast_lio/config/nav2_fastlio_params.yaml` | Path to the Nav2 parameters file |
| `map_file_path` | `str` (file path) | `wheelchair2_navigation/maps/rrc/rrc_lab.yaml` | Path to the map file for Nav2's map_server |
| **Topics** |
| `lidar_cloud_topic` | `str` (topic) | `/livox/lidar` | Topic for the LiDAR point cloud data |
| `fast_lio_odom_topic` | `str` (topic) | `/Odometry` | Topic for the FAST-LIO odometry data |
| `initialpose_topic` | `str` (topic) | `/initialpose` | Topic for the initial pose estimate |
| `localization_pose_topic` | `str` (topic) | `/localization_pose` | Topic for the localization pose output |
| **Localization parameters** |
| `publish_3d_map` | `bool` | `false` | Whether to publish the 3D map for visualization from localization node (saved map from the PCD file) |
| `use_map_cropping` | `bool` | `true` | Whether to crop the map to a specified distance from robot in localization node |
| `map_crop_distance` | `float` | `20.0` | The radius in meters for map cropping in localization node |
| `icp_threshold` | `float` | `0.8` | ICP convergence threshold for localization |
| `icp_max_iterations` | `int` | `50` | Maximum number of ICP iterations for localization |
| `localization_publish_frequency` | `float` | `30.0` | Frequency of publishing localization results |
| **Visualization** |
| `fast_lio_rviz_required` | `bool` | `false` | Whether to launch RViz with `fast_lio` |
| `fast_lio_rviz_config_file` | `str` (file path) | `dependencies/FAST_LIO_ROS2/rviz/fastlio.rviz` | Path to the RViz config file for FAST-LIO |
| `nav2_rviz_required` | `bool` | `false` | Whether to launch RViz for Nav2 visualization |
| `nav2_rviz_config_file` | `str` (file path) | `wheelchair2_navigation/rviz/navigation.rviz` | Path to the RViz config file for Nav2 |

## The `FAST_LIO_ROS2` config file
We've copied the `mid360.yaml` file from the `FAST_LIO_ROS2` package to the `wheelchair2_fast_lio/config` directory as `fast_lio_mapping_mid360.yaml`, but changed some parameters, shown here as a diff and explained below:

```diff
diff --git a/dependencies/FAST_LIO_ROS2/config/mid360.yaml b/wheelchair2_fast_lio/config/fast_lio_mapping_mid360.yaml
@@ -10,14 +10,15 @@
-        map_file_path: "./test.pcd"
+        map_file_path: "your_super_accurate_map.pcd"
 
         common:
-            lid_topic:  "/livox/lidar"
-            imu_topic:  "/livox/imu"
+            lid_topic:  "/livox/lidar_inverted"
+            imu_topic:  "/livox/imu_inverted"
             time_sync_en: false         # ONLY turn on when external time synchronization is really not possible
             time_offset_lidar_to_imu: 0.0 # Time offset between lidar and IMU calibrated by other algorithms, e.g. LI-Init (can be found in README).
                                         # This param will take effect no matter what time_sync_en is. So if the time offset is not known exactly, please set 
as 0.0
 
         preprocess:
-            lidar_type: 1                # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 4 for any other pointcloud input
+            lidar_type: 4               # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 4 for any other pointcloud input
+                                        # we want to use PointCloud2 message as input, so set it to 4
             scan_line:  4
             blind: 0.5
             timestamp_unit: 3
```

Explanation of the changes:
- `map_file_path`: The path to the PCD map file used for localization.
- `lid_topic`: The topic name for the lidar data. We set it to `/livox/lidar_inverted` because we are inverting the data in the `invert_livox_data` node.
- `imu_topic`: The topic name for the IMU data. We set it to `/livox/imu_inverted` because we are inverting the data in the `invert_livox_data` node.
- `lidar_type`: The type of lidar being used. We set it to `4` because we want to use PointCloud2 message as input. The default of `1` uses the custom `livox` message type, which is not considered a standard ROS message type.

## The package parameters YAML file

Additionally, we've added a [`fast_lio_package_params.yaml`](config/fast_lio_package_params.yaml) file to store parameters for the entire package:

This file allows for flexible configuration of the entire `wheelchair2_fast_lio` package and all its components. The priority of the parameters is as follows:
1.  Defaults from the Python launch file.
2.  Parameters from the `fast_lio_package_params.yaml` file.
3.  Parameters from the command line when launching the package.