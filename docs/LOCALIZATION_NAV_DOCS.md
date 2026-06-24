# Localization, Mapping & Navigation — Technical Reference

This document is the full technical reference for the wheelchair2 **3D localization +
odometry + navigation** stack. It covers the three forked upstream modules
(ICP Localization, 3D-BBS, DLIO), their changes from upstream, their ROS 2 interfaces,
and how they feed Nav2.

> **Scope note.** Topics, frames, parameters, and commands below are sourced from the
> actual launch/config/source files in this repository. Anything that could not be
> verified from source is explicitly marked `⚠️ NEEDS VERIFICATION`.

---

## 1. Architecture Overview

The stack splits localization into three cooperating pieces and then hands a stable
`map → odom → base_footprint` TF chain to Nav2.

```
                         Livox MID-360 (3D LiDAR + built-in IMU)
                          │                         │
                 /livox/lidar (PointCloud2)   /livox/imu (Imu)
                          │                         │
        ┌─────────────────┼─────────────────────────┼───────────────────────┐
        │                 │                         │                        │
        ▼                 ▼                         ▼                        ▼
 ┌─────────────┐   ┌──────────────┐         ┌──────────────┐        (optional invert node:
 │    DLIO     │   │   3D-BBS     │         │     ICP      │         /livox/lidar_inverted,
 │  odometry   │   │ global match │         │  refinement  │          /livox/imu_inverted)
 └─────────────┘   └──────────────┘         └──────────────┘
        │                 │                         │
   /Odometry         /global_pose                map→odom TF
   odom→base_         /initialpose  ───────────────┘  (correction)
   footprint TF      (PoseWithCov.)        ▲
        │                 │                │ ICP consumes /Odometry + /initialpose
        │                 └────────────────┘ and refines against the prior PCD map
        │
        ▼
   ┌──────────────────────────────────────────────────────────────────┐
   │                              Nav2                                   │
   │  map_server (2D yaml) · SmacPlanner2D · RotationShimController       │
   │  STVL voxel costmaps · KeepoutFilter · behavior server              │
   │  consumes: map→odom (ICP), odom→base_footprint (DLIO), /points_nonground │
   └──────────────────────────────────────────────────────────────────┘
```

**Data-flow summary:**

| Stage | Module | Provides | TF published |
| :---- | :----- | :------- | :----------- |
| Local odometry | **DLIO** | High-rate `/Odometry` (drifting but smooth) | `odom → base_footprint` |
| Global initialization | **3D-BBS** | One-shot global pose via branch-and-bound matching against the prior PCD map; seeds `/initialpose` | *(none — publishes poses only)* |
| Pose refinement | **ICP** | Continuous correction by ICP-matching live scans to the prior PCD map | `map → odom` |
| Navigation | **Nav2** | Planning, control, costmaps | — |

**Why three modules?** DLIO gives smooth short-term motion but drifts; 3D-BBS recovers a
global pose without a manual 2D-pose-estimate click; ICP keeps the `map→odom` correction
locked onto the prior map over time. Together they replace AMCL for 3D/`mid360`-based runs.

---

## 2. Module Reference

### 2.1 ICP Localization

- **Purpose** — Refines the robot pose by matching the live Livox scan against a prior
  `.pcd` map using libpointmatcher ICP, and broadcasts the `map → odom` correction TF.
- **Upstream repo** — https://github.com/baiyeweiguang/icp_localization_ros2 (branch `main`)
- **Our fork** — https://github.com/Smart-Wheelchair-RRC/icp_localization_ros2 (branch `main`, submodule commit `61c365a`)

#### ICP Changes From Upstream

Seven files changed (`git diff upstream/main...HEAD`). Anchor referenced by all
`[DOCS]` comments in this module: this section.

| File | Symbol / block | What changed | Inferred why |
| :--- | :------------- | :----------- | :----------- |
| `src/ICPlocalization.cpp` | `icpWorker()` | Rewritten from a "run ICP as fast as scans arrive at 100 Hz" loop into a **rate-throttled** loop: ICP only runs once `rate_` seconds have elapsed; between runs the loop re-broadcasts the last TF at 100 Hz. Old body kept commented out below. | Cap ICP CPU cost (esp. on Jetson) while keeping TF fresh so Nav2 never sees a stale transform. |
| `src/ICPlocalization.cpp` | `hasMatchingPointCounts()` (new anon-namespace helper) + guards in `matchScans()` and `icpWorker()` | New validity check: skips clouds whose `features`/`descriptors` column counts mismatch or are empty (non-dense scans). | Prevent `PointMatcher::DataPoints::InvalidField` crashes on malformed Livox frames. |
| `src/ICPlocalization.cpp` | `initialize()` | Adds `icp_localization_ros2.rate` parameter (default `1.0`); adds a 100 Hz wall-timer (`tfRepublishTimer_`) that re-publishes `map→odom` (or `map→range_sensor`) on the executor thread; reads `is_provide_odom_frame` as its **own** parameter; **starts the ICP worker thread here** (moved out of `initializeInternal`). | Decouple TF publish rate from the ICP solve rate; allow `is_provide_odom_frame` independent of `is_use_odometry`. |
| `src/ICPlocalization.cpp` | `initializeInternal()` | Worker-thread creation removed (moved to `initialize()` so `rate_` is set first). | Ordering fix — `rate_` must be read before the worker starts. |
| `src/ICPlocalization.cpp` | `~ICPlocalization()` | Cancels `tfRepublishTimer_` before joining the worker. | Clean shutdown of the new timer. |
| `include/.../ICPlocalization.hpp` | member block | `isFirstScanMatch_` → `std::atomic<bool>`; `isUseOdometry_` default flipped `false → true`; new members `rate_` (=10.0) and `tfRepublishTimer_`; `<atomic>` include added. | Support the threaded TF re-publish and odometry-on-by-default behavior. |
| `src/transform/RigidTransform.cpp` | `interpolate()` | Out-of-range interpolation now logs to `std::cerr` instead of `throw std::runtime_error(...)`. | Stop a transient timestamp gap from killing the node. |
| `src/transform/TfPublisher.cpp` | `publishMapToOdom()` | TF stamp set to `now() + 0.1 s` (was `now()`). | Forward-date the transform so TF lookups slightly into the future succeed (reduces "extrapolation into the future" warnings). |
| `config/icp.yaml` | filters/matcher | `MaxDistDataPointsFilter` enabled @ `30.0`; `RandomSamplingDataPointsFilter.prob` `1.0→0.3`; `KDTreeMatcher` `knn 3→1`, `maxDist 10→3`, `epsilon 1.0→3.16`; `TrimmedDistOutlierFilter.ratio` `0.8→0.65`; `CounterTransformationChecker.maxIterationCount` `40→15`. | Tuned for the MID-360 indoor/ramp maps: fewer points, tighter matching, fewer iterations → faster solve. |
| `config/node_params.yaml` | defaults | `pcd_file_path` → wheelchair lab map; `input_filters_config_path` corrected to `input_filters_mid360.yaml`; `range_data_topic /cloud_registered_body → /livox/lidar`; `imu_data_topic /imu/data → /livox/imu`; `is_provide_odom_frame true → false`. | Point the node at the real robot's topics and map; ICP corrects `map→odom` rather than owning the odom frame. |
| `launch/bringup.launch.py` | `generate_launch_description()` | Adds `period` (loop period, default `3.0`), `pcd_map_path`, and six `initial_pose_*` launch args; switches default input filter to `input_filters_mid360.yaml`; passes `icp_localization_ros2.rate` (typed float) and `pcd_file_path`. | CLI-driven map path, initial pose, and solve rate without editing YAML. |

#### ROS 2 Interface — ICP

**Subscribed topics**

| Topic | Msg type | Description |
| :---- | :------- | :---------- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Live scan (`range_data_topic`). |
| `/livox/imu` | `sensor_msgs/Imu` | IMU for gravity/orientation tracking (`imu_data_topic`). |
| `/Odometry` | `nav_msgs/Odometry` | DLIO odometry prior (`odometry_data_topic`, `is_use_odometry: true`). |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 2D pose estimate / 3D-BBS seed (`set2DPoseCallback`). |

**Published topics**

| Topic | Msg type | Description |
| :---- | :------- | :---------- |
| `range_sensor_pose` | `geometry_msgs/PoseStamped` | Estimated sensor pose (`publishPose()` → `posePub_`). |
| `registered_cloud` | `sensor_msgs/PointCloud2` | Live cloud transformed into the fixed frame (`publishRegisteredCloud()`). |

**TF frames** — Publishes `map → odom` (because `is_provide_odom_frame: false` + `is_use_odometry: true`). `fixed_frame: map`.

**Services / actions** — None on the checked-out commit (`grep` for `create_service` in `src/` returns nothing). *Note:* an `origin/adding-pause-reset-loadmap-services` branch exists upstream-of-fork with pause/reset/load-map services, but it is **not** the commit in use here (`61c365a`).

#### Parameters — ICP

| Parameter | Type | Upstream default | Our override | Description |
| :-------- | :--- | :--------------- | :----------- | :---------- |
| `icp_localization_ros2.range_data_topic` | string | `/cloud_registered_body` | `/livox/lidar` | Input cloud topic. |
| `icp_localization_ros2.imu_data_topic` | string | `/imu/data` | `/livox/imu` | Input IMU topic. |
| `icp_localization_ros2.odometry_data_topic` | string | `/Odometry` | `/Odometry` | Odometry prior. |
| `icp_localization_ros2.is_use_odometry` | bool | `false` | `true` | Use odometry prior. |
| `icp_localization_ros2.is_provide_odom_frame` | bool | `true` | `false` | If false, publishes `map→odom` (not `map→range_sensor`). |
| `icp_localization_ros2.rate` | double | *(n/a — new)* | `1.0` (node) / `10.0` (header) / `2.0`–`10.0` (launch `period`) | ICP solve period in **seconds**. |
| `icp_localization_ros2.fixed_frame` | string | `map` | `map` | World frame. |
| `pcd_file_path` | string | `/home/fyt/...test.pcd` | lab map `.pcd` | Prior map. |
| `input_filters_config_path` | string | `input_filteres_mid360.yaml` *(typo)* | `input_filters_mid360.yaml` | Input filter chain. |

#### Running ICP Individually

```bash
# Refinement only, against an explicit PCD map, 2 s solve period
ros2 launch icp_localization_ros2 bringup.launch.py \
  pcd_map_path:=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd \
  period:=2.0
```
Expect: `map → odom` on `/tf`, console `Scan matching took: <ms>`. Requires `/livox/lidar`,
`/livox/imu`, and `/Odometry` to be live (run DLIO first), plus an `/initialpose`.

---

### 2.2 3D-BBS

- **Purpose** — Global, click-free initial localization. Runs GPU branch-and-bound 3D
  scan matching (`gpu_bbs3d`) of the live scan against the prior map, then publishes the
  recovered global pose and seeds `/initialpose` for ICP/Nav2.
- **Upstream repo** — https://github.com/KOKIAOKI/3d_bbs (branch `main`)
- **Our fork** — https://github.com/Smart-Wheelchair-RRC/RRC-3DLocalization.git (branch `main`, submodule `dependencies/3DLocalization`, commit `4054fd6`)

#### 3D BBS Changes From Upstream

> **⚠️ Correction to the original brief.** The brief stated *"3D BBS has not had its source
> code modified — only the directory structure changed."* That is **not** accurate. The
> directory restructuring is real (documented below), **but the ROS 2 wrapper source was
> also functionally changed.** Both are documented here. Note: the BBS *algorithm* itself
> (`gpu_bbs3d` core library) is **not** vendored in this fork — it is resolved at build time
> via `find_package(gpu_bbs3d)` (`cmake/Findgpu_bbs3d.cmake`), so no algorithmic change was
> possible here regardless.

**A. Directory / structural reorganization.** Upstream `3d_bbs` is a monorepo
(`bbs3d/` core lib + `ros2_test/{click_loc,rviz2,iridescence}` + `test/` + `docker/`).
This fork keeps only the ROS 2 RViz wrapper and the click helper, promoted to top-level
ament packages:

| Upstream path | Our path | Change |
| :------------ | :------- | :----- |
| `ros2_test/rviz2/` (package `ros2_test_rviz2`) | `wheelchair2_3dbbs/` | Package renamed → `wheelchair2_3dbbs`. |
| `ros2_test/rviz2/src/gpu_bbs3d_rviz2/` | `wheelchair2_3dbbs/src/gpu_3dbbs/` | Source dir renamed (file names kept). |
| `ros2_test/click_loc/` (package `click_loc`) | `click_save/` | Package renamed → `click_save` (top-level). |
| `bbs3d/`, `ros2_test/iridescence/`, `test/`, `docker/`, `figs/` | *(removed)* | Dropped; core lib pulled via `find_package(gpu_bbs3d)`. |
| — | `wheelchair2_3dbbs/launch/3dbbs.launch.py` | **New** wrapper launch (no upstream equivalent). |

- **`CMakeLists.txt`**: `project(ros2_test_rviz2)` → `project(wheelchair2_3dbbs)`; executable
  `gpu_ros2_test_rviz2` → `gpu_3dbbs`; `ament_auto_package` now also installs `config` and
  uses `USE_SCOPED_HEADER_INSTALL_DIR`; added explicit `install(PROGRAMS scripts/click_loc.py …)`.
- **`package.xml`**: `<package>` → `<package format="3">`; name → `wheelchair2_3dbbs`;
  added `exec_depend` on `rclpy`, `launch`, `launch_ros`.
- **`launch/gpu_ros2_test_rviz2_launch.py`**: package/executable references renamed; config
  and rviz paths resolved via `get_package_share_directory` instead of relative paths.
- **`config/ros2_test.yaml`**: `lidar_topic_name /livox/points → /livox/lidar`;
  `tar_leaf_size 0.1 → 0.5`; `src_leaf_size 2.0 → 0.1`; `target_clouds` default path changed
  (overridden at launch anyway).

**B. Source-code (wrapper) behavior changes** — verified by diffing against
`bbs_upstream/main:ros2_test/rviz2/...`. *(Per project decision, these source files carry
documentation here but are intentionally **not** annotated with in-code `[DOCS]` comments.)*

| File | What changed | Inferred why |
| :--- | :----------- | :----------- |
| `src/gpu_3dbbs/gpu_ros2_test_rviz2.cpp` | (1) New `initialpose_pub_` publisher to `/initialpose`. (2) **Auto-trigger thread** in constructor: after a 2 s sleep, synthesizes `click_callback(true)` — self-localizes on startup without a manual GUI click. (3) After a successful global match, publishes the estimated pose to `/initialpose` (`PoseWithCovarianceStamped`) with fixed covariance (0.25 m position, 0.0685 rad rotation). | Make global init automatic and seed the downstream localizer (ICP/Nav2) so the operator need not click "localize." |
| `include/ros2_test_rviz2.hpp` | Adds the `/initialpose` publisher member + `geometry_msgs/PoseWithCovarianceStamped` include. | Declares the above. |
| `include/load_rviz2.hpp` | Adds a debug `std::cout` of `min_level_res`. | Diagnostics. |

#### ROS 2 Interface — 3D-BBS

**Subscribed topics** *(from `gpu_ros2_test_rviz2.cpp`)*

| Topic | Msg type | Description |
| :---- | :------- | :---------- |
| `/click_loc` | `std_msgs/Bool` | Trigger a global localization attempt (also auto-fired 2 s after startup). |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Source scan (`lidar_topic_name`). |
| `/livox/imu` | `sensor_msgs/Imu` | IMU for gravity alignment (`imu_topic_name`). |

**Published topics**

| Topic | Msg type | Description |
| :---- | :------- | :---------- |
| `/global_pose` | `geometry_msgs/PoseStamped` | Recovered global pose from BBS matching. |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Seed for ICP/Nav2 (fork addition). |
| `/tar_points` | `sensor_msgs/PointCloud2` | Target (map) cloud for RViz. |
| `/src_points_on_global_pose` | `sensor_msgs/PointCloud2` | Source cloud transformed onto the global pose. |
| `/score` | `std_msgs/Int32` | BBS match score. |
| `/time` | `std_msgs/Float32` | Match compute time. |

**TF frames** — None published by this node (pose-only). Match is computed in `map`.

**Services / actions** — None.

#### Parameters — 3D-BBS (`config/ros2_test.yaml`)

| Parameter | Type | Upstream default | Our override | Description |
| :-------- | :--- | :--------------- | :----------- | :---------- |
| `target_clouds` | string | `your_path/target` | set at launch from `dirname(pcd_map_path)` | Folder holding the target map cloud. |
| `lidar_topic_name` | string | `/livox/points` | `/livox/lidar` | Source scan topic. |
| `imu_topic_name` | string | `/livox/imu` | `/livox/imu` | IMU topic. |
| `min_level_res` | double | `1.0` | `0.5` indoor / `1.0` outdoor (set by `3dbbs.launch.py` from `nature`) | BBS minimum voxel resolution. |
| `max_level` | int | `6` | `6` | BBS octree depth. |
| `min_rpy` / `max_rpy` | float[3] | `[-0.02,-0.02,0.0]` / `[0.02,0.02,6.28]` | same | Angular search range (rad). |
| `score_threshold_percentage` | double | `0.9` | `0.9` | Accept threshold. |
| `tar_leaf_size` | double | `0.1` | `0.5` | Target downsample (m). |
| `src_leaf_size` | double | `2.0` | `0.1` | Source downsample (m). |
| `max_scan_range` | double | `100.0` | `100.0` | Source crop (m). |
| `timeout_msec` | int | `0` | `0` | Match timeout (0 = off). |

#### Running 3D-BBS Individually

```bash
# nature and pcd_map_path are MANDATORY
ros2 launch wheelchair2_3dbbs 3dbbs.launch.py \
  nature:=indoor \
  pcd_map_path:=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd
```
Expect: RViz opens (see Common Issues #4 — `3dbbs_rviz` defaults `True`); `/global_pose`,
`/initialpose`, `/score`, `/time` appear; an auto-localize fires ~2 s after the node starts.

---

### 2.3 DLIO (Direct LiDAR-Inertial Odometry)

- **Purpose** — High-rate local odometry. Fuses Livox scans + IMU to produce smooth
  `/Odometry` and the `odom → base_footprint` TF that everything downstream rides on.
- **Upstream repo** — https://github.com/vectr-ucla/direct_lidar_inertial_odometry
  (branch `feature/ros2` — **the true fork point**; not `master`, which is ROS 1)
- **Our fork** — https://github.com/Smart-Wheelchair-RRC/direct_lidar_inertial_odometry
  (branch `feature/ros2`, submodule commit `cf711ee`)

#### DLIO Changes From Upstream

Only **2 commits** ahead of `upstream/feature/ros2`; 4 files. (Diffing against `master`
would wrongly attribute the entire ROS 1→ROS 2 port to this fork — that work is upstream's.)

| File | Symbol / block | What changed | Inferred why |
| :--- | :------------- | :----------- | :----------- |
| `src/dlio/odom.cc` | `dlio::OdomNode::debug()` | Entire terminal status-print body **commented out** (the only source change; `git diff` shows equal +/- lines = comment-out). | Silence the full-screen ANSI dashboard during multi-pane tmux runs / reduce console churn on Jetson. |
| `cfg/params.yaml` | params | `use_sim_time true → false`; `frames/baselink base_link → base_footprint`; `frames/lidar lidar → livox/lidar`; `frames/imu imu → livox/imu`; `odom/gravity 9.80665 → 0.0`. | Match the real robot's frame names; **`gravity: 0.0`** — ⚠️ unusual; likely a ramp-test workaround. See Risk table. |
| `cfg/dlio.yaml` | `extrinsics/baselink2imu/t` | `[0.006253,-0.011775,0.007645] → [-0.011,-0.02329,0.04419]` | Real measured base_link→IMU translation for this chassis. |
| `launch/dlio.launch.py` | defaults + remap | `pointcloud_topic points_raw → /livox/lidar`; `imu_topic imu_raw → /livox/imu`; output `odom` remapped `dlio/odom_node/odom → /Odometry`. | Wire DLIO to the Livox driver and to the topic ICP/Nav2 expect. |

#### ROS 2 Interface — DLIO

**Subscribed topics** *(after launch remaps)*

| Topic | Msg type | Description |
| :---- | :------- | :---------- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Input scan (`pointcloud`). |
| `/livox/imu` | `sensor_msgs/Imu` | Input IMU (`imu`). |

**Published topics** *(after launch remaps)*

| Topic | Msg type | Description |
| :---- | :------- | :---------- |
| `/Odometry` | `nav_msgs/Odometry` | Primary odometry output (`odom`, consumed by ICP). |
| `dlio/odom_node/pose` | `geometry_msgs/PoseStamped` | Current pose (`pose`). |
| `dlio/odom_node/path` | `nav_msgs/Path` | Trajectory (`path`). |
| `dlio/odom_node/keyframes` | `geometry_msgs/PoseArray` | Keyframe poses (`kf_pose`). |
| `.../pointcloud/keyframe`, `.../pointcloud/deskewed` | `sensor_msgs/PointCloud2` | Keyframe + deskewed clouds (`kf_cloud`, `deskewed`). |

**TF frames** — Publishes `odom → base_footprint` (`frames/odom: odom`, `frames/baselink: base_footprint`).

**Services** — `dlio_map_node` advertises the **`save_pcd`** service (type `direct_lidar_inertial_odometry/srv/SavePCD`; the `.srv` file was renamed from upstream `save_pcd.srv` → `SavePCD.srv`, rename only, no body change).

#### Parameters — DLIO

| Parameter | Type | Upstream default | Our override | Description |
| :-------- | :--- | :--------------- | :----------- | :---------- |
| `use_sim_time` | bool | `true` | `false` | Real-hardware clock. |
| `frames/odom` | string | `odom` | `odom` | Odom frame. |
| `frames/baselink` | string | `base_link` | `base_footprint` | Body frame. |
| `frames/lidar` | string | `lidar` | `livox/lidar` | LiDAR frame. |
| `frames/imu` | string | `imu` | `livox/imu` | IMU frame. |
| `odom/gravity` | double | `9.80665` | `0.0` | Gravity magnitude — see Risk table. |
| `extrinsics/baselink2imu/t` | double[3] | `[0.006253,-0.011775,0.007645]` | `[-0.011,-0.02329,0.04419]` | base→IMU translation. |
| `pointcloud_topic` (launch) | string | `points_raw` | `/livox/lidar` | Input cloud. |
| `imu_topic` (launch) | string | `imu_raw` | `/livox/imu` | Input IMU. |

#### Running DLIO Individually

```bash
# Odometry only, no RViz, real-hardware clock
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false use_sim_time:=False
```
Expect: `/Odometry` (`nav_msgs/Odometry`) and `odom → base_footprint` on `/tf`.
Requires `/livox/lidar` and `/livox/imu` live (run the Livox driver first).

---

## 3. Nav2 Integration

Nav2 is launched separately from localization via
`wheelchair2_navigation/launch/nav2_3d.launch.py` (config: `config/nav2_params.yaml`).

**How localization feeds Nav2.** Nav2 does **not** run AMCL in the 3D stack — the
`map → odom` correction comes from **ICP** and `odom → base_footprint` from **DLIO**.
(The `amcl` block still exists in `nav2_params.yaml` and is used by the separate
`nav2_amcl.launch.py` 2D pipeline.) Nav2's costmaps consume the resulting TF chain plus a
non-ground point cloud for obstacles.

**Active Nav2 plugins** *(from `nav2_params.yaml`)*

| Server | Plugin |
| :----- | :----- |
| Planner (`GridBased`) | `nav2_smac_planner/SmacPlanner2D` |
| Controller (`FollowPath`) | `nav2_rotation_shim_controller::RotationShimController` |
| Progress checker | `nav2_controller::SimpleProgressChecker` |
| Goal checker | `nav2_controller::SimpleGoalChecker` |
| Local costmap layers | `lidar_obstacle_layer` (`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`), `inflation_layer` |
| Global costmap layers | `static_layer`, `stvl_layer` (STVL), `inflation_layer` |
| Costmap filter | `nav2_costmap_2d::KeepoutFilter` (`/costmap_filter_info`) |
| Behaviors | `Spin`, `BackUp`, `DriveOnHeading`, `Wait`, `AssistedTeleop` |
| BT navigators | `NavigateToPoseNavigator`, `NavigateThroughPosesNavigator` |
| Collision monitor | `collision_monitor` (`cmd_vel_smoothed → cmd_vel`, `state_topic: collision_monitor_state`) |

**Costmap obstacle source** — both STVL layers use `observation_sources` on
`/points_nonground` (`PointCloud2`). ⚠️ NEEDS VERIFICATION: which node publishes
`/points_nonground` (likely a ground-segmentation/box-filter node; `box_filter.py` is launched
by `nav2_3d.launch.py` but its output topic was not confirmed from source).

**Key frames** — `global_frame: map`, `robot_base_frame: base_footprint`,
local costmap `global_frame: odom`.

**Notable parameter overrides vs Nav2 defaults**

| Area | Setting | Note |
| :--- | :------ | :--- |
| Controller | `RotationShimController` wrapping `FollowPath` | Rotates to heading before path-following. |
| Planner | `SmacPlanner2D` | 2D Smac (not the default NavFn/Hybrid). |
| Costmaps | STVL voxel layers | Spatio-temporal voxel layer instead of default obstacle/voxel layer. |
| Filters | `KeepoutFilter` on both costmaps | Driven by `keepout_params.yaml` + mask, gated by `use_keepout_zone` (default `false`). |
| AMCL model | `nav2_amcl::DifferentialMotionModel` | Only relevant to the 2D `nav2_amcl.launch.py` path. |

Full per-parameter values: see `wheelchair2_navigation/config/nav2_params.yaml`.

---

## 4. Divergence Table & Maintenance Notes

At-a-glance reference for maintainers pulling future upstream changes. "Upstream link" base
paths: ICP `…/baiyeweiguang/icp_localization_ros2/blob/main/`, DLIO
`…/vectr-ucla/direct_lidar_inertial_odometry/blob/feature/ros2/`, 3D-BBS
`…/KOKIAOKI/3d_bbs/blob/main/`.

| Module | File | What Changed | Upstream Link | Risk if Upstream Updates |
| :----- | :--- | :----------- | :------------ | :----------------------- |
| ICP | `src/ICPlocalization.cpp` | `icpWorker()` rewritten (rate throttle + TF re-publish); cloud validity guard; worker start moved to `initialize()`; `rate_` param | `src/ICPlocalization.cpp` | **High** — large logic rewrite; upstream edits to the worker/threading will conflict. Re-apply manually. |
| ICP | `include/.../ICPlocalization.hpp` | atomic flag, `rate_`, `tfRepublishTimer_`, `isUseOdometry_` default flip | `include/icp_localization_ros2/ICPlocalization.hpp` | Medium — header layout changes will conflict. |
| ICP | `src/transform/TfPublisher.cpp` | `+0.1 s` TF stamp | `src/transform/TfPublisher.cpp` | Low — one-line; easy to re-apply. |
| ICP | `src/transform/RigidTransform.cpp` | throw → cerr | `src/transform/RigidTransform.cpp` | Low. |
| ICP | `config/icp.yaml`, `config/node_params.yaml` | tuned filters/topics/map | `config/*.yaml` | Medium — re-tune if upstream changes filter schema. |
| ICP | `launch/bringup.launch.py` | new launch args (`period`, `pcd_map_path`, `initial_pose_*`) | `launch/bringup.launch.py` | Medium. |
| 3D-BBS | `src/gpu_3dbbs/gpu_ros2_test_rviz2.cpp` | auto-localize thread + `/initialpose` seeding | `ros2_test/rviz2/src/gpu_bbs3d_rviz2/gpu_ros2_test_rviz2.cpp` | **High** — wrapper diverged *and* lives at a renamed path; upstream changes won't merge cleanly. |
| 3D-BBS | `CMakeLists.txt`, `package.xml`, `launch/*`, `config/ros2_test.yaml` | package rename + path/topic/leaf-size changes | `ros2_test/rviz2/*` | Medium — structural; the `bbs3d` core is external (`find_package(gpu_bbs3d)`), so core upgrades are decoupled. |
| DLIO | `src/dlio/odom.cc` | `debug()` body commented out | `src/dlio/odom.cc#L1887` | Low — re-comment after pull (or restore + redirect logging). |
| DLIO | `cfg/params.yaml` | frames + `gravity: 0.0` | `cfg/params.yaml` | **High (correctness)** — `odom/gravity: 0.0` disables gravity in the IMU model; verify this is intended before any upstream IMU/gravity changes are merged. |
| DLIO | `cfg/dlio.yaml` | base→IMU extrinsics | `cfg/dlio.yaml` | Low — robot-specific calibration; keep ours. |
| DLIO | `launch/dlio.launch.py` | topic defaults + `odom→/Odometry` remap | `launch/dlio.launch.py` | Low/Medium. |

**Keeping docs in sync:** after pulling any upstream change, re-run the Step-2 diff
(`git fetch upstream && git diff upstream/<branch>...HEAD`) for the affected submodule and
update the relevant "Changes From Upstream" subsection here, plus the `[DOCS]` anchors in
the touched files (anchors: `#icp-changes-from-upstream`, `#dlio-changes-from-upstream`,
`#3d-bbs-changes-from-upstream`).
