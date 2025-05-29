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

Finally, make sure to build the package:

```bash
colcon build --symlink-install
```

## Available Nodes

| Node | Description |
| --- | --- |
| `invert_livox_data` | Inverts the lidar and IMU data from the `livox` driver. |
| `tf_fast_lio_init_frame` | Publishes the tf from `map` to `camera_init`. |
| `localization` | Localization using a pre-built PCD map. |
| `fastlio_mapping` (from the `FAST_LIO_ROS2` package) | The main FAST-LIO node that performs the odometry. |

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
5. `rviz2`: This node launches RViz with the specified config file. It is launched last to ensure that all the other nodes are running before launching RViz.

### Parameters

The following parameters are available in the launch file:

| Parameter | Expected Type | Default Value | Description |
| --- | --- | --- | --- |
| `lidar_inverted` | `bool` | `true` | Whether to invert the lidar data |
| `init_frame_tf_required` | `bool` | `true` | Whether to publish tf from `odom` to `camera_init` |
| `fast_lio_config` | `str` (file name, not path) | `fast_lio_mapping_mid360.yaml` | Name of the `fast_lio` config file |
| `fast_lio_config_path` | `str` (folder path) | `wheelchair2_fast_lio/config` | Path to the `fast_lio` config file directory |
| `use_sim_time` | `bool` | `false` | Whether to use simulation time |
| `fast_lio_rviz_required` | `bool` | `false` | Whether to launch RViz with `fast_lio` |
| `fast_lio_rviz_config_file` | `str` (file path) | `dependencies/FAST_LIO_ROS2/rviz/fastlio.rviz` | Path to the RViz config file |
| `use_localization` | `bool` | `true` | Whether to use localization mode |
| `pcd_map_path` | `str` (file path) | `you-super-accurate-map.pcd` | Path to the PCD map file |
| `lidar_cloud_topic` | `str` (topic) | `/livox/lidar` | Topic for the LiDAR point cloud data |
| `fast_lio_odom_topic` | `str` (topic) | `/Odometry` | Topic for the FAST-LIO odometry data |
| `initialpose_topic` | `str` (topic) | `/initialpose` | Topic for the initial pose estimate |
| `localization_pose_topic` | `str` (topic) | `/localization_pose` | Topic for the localization pose output |

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

Additionally, we've added a `fast_lio_package_params.yaml` file to store parameters for the entire package:

```yaml
# Configuration parameters for wheelchair2_fast_lio package

lidar:
    inverted: true

tf:
    init_frame_required: true

fast_lio:
    config_file: "fast_lio_mapping_mid360.yaml" # name of the config file
    config_path: "" # use default path (wheelchair2_fast_lio/config)
    rviz:
        required: false
        config_file: "" # use default config file (dependencies/FAST_LIO_ROS2/rviz/fastlio.rviz)

use_sim_time: false

localization:
    enabled: true

    # IRL map
    # pcd_map_path: "$MAP_DIR/rrc/rrc_lab.pcd"

    # simulation map
    # pcd_map_path: "$MAP_DIR/sim/obstacles_world.pcd"

topics:
    lidar_cloud: "/livox/lidar"
    fast_lio_odom: "/Odometry"
    initialpose: "/initialpose"
    localization_pose: "/localization_pose"
```

This file allows for flexible configuration of the entire `wheelchair2_fast_lio` package and all its components. The priority of the parameters is as follows:
1.  Defaults from the Python launch file.
2.  Parameters from the `fast_lio_package_params.yaml` file.
3.  Parameters from the command line when launching the package.