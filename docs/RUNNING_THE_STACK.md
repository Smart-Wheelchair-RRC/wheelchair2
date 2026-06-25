# Running the Stack — Operator Runbook

Copy-paste runbook for the wheelchair2 3D localization + mapping + navigation stack.
No assumed context — every command below is sourced from a real file in this repo.

> All paths assume the in-container workspace root `/home/container_user/wheelchair2`
> and that the workspace has been built and sourced (see §1). `$MAP_DIR` is exported by
> the dev-container environment and points at the maps directory.

---

## 1. Prerequisites

**ROS 2 distro** — ROS 2 **Humble** (with Gazebo Harmonic for simulation). Run inside the
project dev-container (`.devcontainer/`), which provides ROS 2, CUDA (for 3D-BBS GPU),
`gpu_bbs3d`, and the `wss`/`wsb`/`wsd` aliases.

**Environment variables**

| Var | Purpose | How to set |
| :-- | :------ | :--------- |
| `ROS_DOMAIN_ID` | DDS domain isolation (0–101) | Set in `.devcontainer/devcontainer.json` (`containerEnv.ROS_DOMAIN_ID`). |
| `MAP_DIR` | Base path for `.pcd` / `.yaml` maps | Provided by the dev-container env. ⚠️ NEEDS VERIFICATION (exact export location). |
| `DISPLAY` | RViz / GUI X11 | Set by the container; `nav_3D_loc.tmux` forces `export DISPLAY=:1`. |

**Hardware** (inferred from launch/config)

| Device | Model | Notes |
| :----- | :---- | :---- |
| 3D LiDAR + IMU | **Livox MID-360** | Publishes `/livox/lidar` + `/livox/imu`. Static IP `192.168.1.50/24`, network profile "Livox". |
| Motor controller | Kangaroo X2 + Sabertooth 2x32 | On `/dev/ttyACM0` (`sudo chmod 777 /dev/ttyACM0`). |
| 2D LiDAR | RPLidar | On `/dev/ttyUSB0` (`sudo chmod 777 /dev/ttyUSB0`); used for ramp nav / 2D costmap. |
| RGB-D camera | Realsense D455 | RTABMAP only; not part of the 3D-BBS/ICP/DLIO path. |

**Build the workspace**

```bash
# Full build (alias 'wsb'):
cd /home/container_user/wheelchair2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash      # alias 'wss'
# Clean build artifacts (alias 'wsd'):
# rm -rf build install log
```

---

## 2. Running the Full Stack

The full **localization + navigation** pipeline on the real wheelchair is two launches
(this is exactly what the `nav_3D_loc.tmux` session automates — see §3):

```bash
# --- choose your map (these are the values used in nav_3D_loc.tmux) ---
export PCD_PATH=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd
export YAML_PATH=$MAP_DIR/rrc/rrc_lab.yaml

# 1) Localization stack: DLIO (odometry) + 3D-BBS (global) + ICP (refine)
ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=indoor

# 2) Navigation (in a second terminal, after ~7 s)
ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$YAML_PATH
```

You also need bringup + sensors running first (controllers, Livox, optional RPLidar):

```bash
sudo chmod 777 /dev/ttyACM0
ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=False
ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=False
```

**Expected confirmation it started correctly**
- `ros2 topic echo /Odometry --once` returns a message (DLIO up).
- `ros2 run tf2_ros tf2_echo map base_footprint` resolves (ICP `map→odom` + DLIO `odom→base_footprint`).
- Console shows `Scan matching took: <ms>` (ICP) and `/global_pose` publishes once after BBS converges.
- RViz shows the robot localized on the map; Nav2 "Active" once lifecycle managers finish.

**Stop everything cleanly**
- If launched in plain terminals: `Ctrl-C` in each.
- If launched via tmux (§3): `tmux kill-session -t wheelchair_navigation` (or the relevant session name).

---

## 3. Tmux Session Reference

Three tmux session files exist in the repo (found via `find` + `tmux new-session` grep).
None use tmuxinator.

### Session: `wheelchair_navigation`

- **Purpose**: Full real-wheelchair 3D localization + navigation + GUI run.
- **File**: `wheelchair2_navigation/shell_scripts/nav_3D_loc.tmux`
- **How to start**: `bash wheelchair2_navigation/shell_scripts/nav_3D_loc.tmux`
- **How to stop**: `tmux kill-session -t wheelchair_navigation`
- **Map selection**: edit the `PCD_PATH` / `YAML_PATH` / `NATURE` block at the top before starting (defaults to the RRC lab map, `indoor`).

| Window | Pane | Command | What it does |
| :----- | :--- | :------ | :----------- |
| 0 `bringup` | 0.0 | `ros2 launch wheelchair2_bringup livox.launch.py ramp_nav:=True` | Starts the Livox MID-360 driver → `/livox/lidar`, `/livox/imu`. `ramp_nav:=True` enables ramp-mode handling. |
| 0 `bringup` | 0.1 | `sudo chmod 777 /dev/ttyACM0 && sleep 2 && ros2 launch wheelchair2_bringup wheelchair2.launch.py robot_name:=blacky use_sim_time:=False` | Grants Kangaroo X2 serial perms, then brings up controllers/robot description for wheelchair "blacky". |
| 0 `bringup` | 0.2 | `sudo chmod 777 /dev/ttyUSB0 && sleep 2 && ros2 launch wheelchair2_bringup rplidar.launch.py use_sim_time:=False` | Grants RPLidar serial perms, then starts the 2D LiDAR (used by costmap/ramp nav). |
| 1 `loc_nav` | 1.0 | `ros2 launch wheelchair2_localization 3d_stack.launch.py pcd_map_path:=$PCD_PATH nature:=$NATURE \| tee loc.log` | **Localization stack**: DLIO (`/Odometry`, `odom→base_footprint`) + 3D-BBS (`/global_pose`, `/initialpose`) + ICP (`map→odom`). Logs to `loc.log`. |
| 1 `loc_nav` | 1.1 | `sleep 7 && ros2 launch wheelchair2_navigation nav2_3d.launch.py map:=$YAML_PATH \| tee nav.log` | **Nav2** (after 7 s so TF is up): map_server + SmacPlanner2D + RotationShim + STVL costmaps. Logs to `nav.log`. |
| 2 `app` | 2.0 | `ros2 launch wheelchair2_gui wheelchair_gui.launch.py \| tee gui.log` | Operator GUI / ROS bridge. |
| 2 `app` | 2.1 | `ros2 run wheelchair2_control keyboard_teleop_constant.py` | Constant-velocity keyboard teleop (manual override). |

- **Notes**: Pane 1.1 sleeps 7 s so localization TF (`map→odom`, `odom→base_footprint`) exists before Nav2 starts — **do not remove this delay**. A commented-out optional pane runs `click_loc.py` for manual 3D-BBS re-localization. Window 0 uses a `tiled` layout.

### Session: `3dbbs_icp_fastlio`

- **Purpose**: Bring-up + (originally) the 3D-BBS / ICP / FastLIO localization+mapping bench; in the current file most localization panes are commented out, leaving bringup + LiDAR + two helper scripts.
- **File**: `wheelchair2_localization/scripts/3dbbs_icp_fastlio.tmux`
- **How to start**: `bash wheelchair2_localization/scripts/3dbbs_icp_fastlio.tmux`
- **How to stop**: `tmux kill-session -t 3dbbs_icp_fastlio`

| Window | Pane | Command | What it does |
| :----- | :--- | :------ | :----------- |
| 0 | 0.0 | `ros2 launch wheelchair2_bringup wheelchair2.launch.py use_sim_time:=False` | Robot bringup (controllers, description). |
| 0 | 0.1 | `ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=False` | Livox MID-360 driver → `/livox/lidar`, `/livox/imu`. |
| 0 | 0.5 | `python3 .../RRC-3DLocalization/wheelchair2_3dbbs/scripts/click_loc.py` | Tk "localize" button → publishes `/click_loc` (`Bool`) to trigger 3D-BBS global matching. ⚠️ NEEDS VERIFICATION (path uses `src/RRC-3DLocalization/…`; submodule is `dependencies/3DLocalization/…`). |
| 0 | 0.6 | `python3 .../wheelchair2_bringup/scripts/invert_lidar.py` | Inverts Livox data (180° about X) → `/livox/*_inverted`. ⚠️ NEEDS VERIFICATION (`invert_lidar.py` not found under `wheelchair2_bringup`; the equivalent node `invert_livox_data` lives in `wheelchair2_localization`). |

- **Notes**: Panes 2–4 (3D-BBS launch, FastLIO mapping, ICP `bringup.launch.py rate:=10.0`) are **commented out** in the current file — this session no longer launches the full localization chain on its own. The split-window targets (`0.4`, `0.5`) reference panes created by the commented blocks, so pane numbering may not behave as written if those blocks stay disabled. Prefer the `wheelchair_navigation` session for a real run.

### Session: `navigation_sim`

- **Purpose**: Full Gazebo **simulation** navigation (2D AMCL-based).
- **File**: `wheelchair2_navigation/shell_scripts/sim_navigation.tmux`
- **How to start**: `bash wheelchair2_navigation/shell_scripts/sim_navigation.tmux`
- **How to stop**: `tmux kill-session -t navigation_sim`

| Window | Pane | Command | What it does |
| :----- | :--- | :------ | :----------- |
| 0 `gazebo_sim` | — | `ros2 launch wheelchair2_gazebo wheelchair2_sim.launch.py use_sim_time:=true use_joy:=true` | Gazebo Harmonic sim + robot + joystick. |
| 1 `livox_lidar` | — | `ros2 launch wheelchair2_bringup livox.launch.py use_sim_time:=true` | Livox driver bridged to sim clock. |
| 2 `nav2` | — | `ros2 launch wheelchair2_navigation nav2_amcl.launch.py use_sim_time:=true map:=$MAP_DIR/sim/obstacles_world.yaml` | Nav2 + **AMCL** 2D localization against the sim map. |
| 3 `teleop` | — | `ros2 run wheelchair2_control keyboard_teleop_constant.py` | Keyboard teleop. |

- **Notes**: This is the **2D/AMCL** sim pipeline — it does **not** use DLIO/3D-BBS/ICP. The script `sleep`s 10 s after Gazebo and 5 s after Livox so each layer comes up in order. A commented-out keepout-zone window exists.

---

## 4. Running Individual Modules

#### ICP Localization
```bash
ros2 launch icp_localization_ros2 bringup.launch.py \
  pcd_map_path:=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd period:=2.0
```
Starts the ICP localizer. Publishes `map → odom` TF (after first match). **Needs**
`/livox/lidar`, `/livox/imu`, `/Odometry` (run DLIO first) and an `/initialpose`.

#### 3D BBS
```bash
ros2 launch wheelchair2_3dbbs 3dbbs.launch.py \
  nature:=indoor pcd_map_path:=$MAP_DIR/rrc/lab_map_with_outside_corridor.pcd
```
Starts the GPU BBS global matcher (+ RViz). Publishes `/global_pose`, `/initialpose`,
`/score`, `/time`. Auto-localizes ~2 s after start; or press the `click_loc.py` button.

#### DLIO
```bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false use_sim_time:=False
```
Starts the odometry node. Publishes `/Odometry` and `odom → base_footprint` TF.
**Needs** `/livox/lidar` + `/livox/imu` (run the Livox driver first).

---

## 5. Common Issues & Quick Fixes

1. **No `map → odom` TF / robot not localized**
   → *Likely cause:* ICP never matched — `/Odometry` or `/initialpose` missing.
   → *Fix:* Confirm DLIO is up (`ros2 topic echo /Odometry --once`); let 3D-BBS auto-seed
   `/initialpose`, or press the `click_loc.py` "localize" button, or publish a 2D Pose
   Estimate in RViz.

2. **`Permission denied` on `/dev/ttyACM0` or `/dev/ttyUSB0`**
   → *Cause:* serial device perms not set.
   → *Fix:* `sudo chmod 777 /dev/ttyACM0` (motor controller)
   before bringup — the tmux scripts already do this.

3. **DLIO drifts vertically / odd pitch on ramps**
   → *Cause:* `cfg/params.yaml` sets `odom/gravity: 0.0` (overrides the upstream `9.80665`).
   → *Fix:* This is an intentional fork override for ramp tests — verify it is what you want;
   restore `9.80665` for normal flat-floor IMU gravity modeling. (See Risk table in
   `LOCALIZATION_NAV_DOCS.md#4-divergence-table--maintenance-notes`.)

4. **RViz opens unexpectedly when launching 3D-BBS from the full stack**
   → *Cause:* `3dbbs.launch.py` declares `3dbbs_rviz` (default `True`), but `3d_stack.launch.py`
   passes `rviz:=false` — an **undeclared** arg for this file, so it's ignored and RViz still opens.
   → *Fix:* pass `3dbbs_rviz:=False` to suppress it.

5. **3D-BBS finds no map / empty `/tar_points`**
   → *Cause:* `target_clouds` is derived from `dirname(pcd_map_path)`; the `.pcd` must live in
   that directory and be loadable.
   → *Fix:* Pass an absolute `pcd_map_path` to a real `.pcd`; check the file exists in-container.

6. **Nav2 costmap has no obstacles**
   → *Cause:* STVL layers subscribe to `/points_nonground`, which must be published by the
   ground-segmentation/box-filter node.
   → *Fix:* Confirm that topic is live (`ros2 topic hz /points_nonground`).
   -> /points_nonground and Staircase nodes are published by the ```ground_detection_resuse_planes.cpp``` node in wheelchair2_bringup package.

7. **Frame mismatch (`base_link` vs `base_footprint`)**
   → *Cause:* DLIO/Nav2 use `base_footprint`; a node still configured for upstream `base_link`
   will not connect to the TF tree.
   → *Fix:* Ensure all consumers use `base_footprint` (the fork default).

8. **Wrong topics (`points_raw` / `imu_raw` / `/livox/points`)**
   → *Cause:* a stale upstream default leaked through.
   → *Fix:* The fork standardizes on `/livox/lidar` + `/livox/imu`; check launch remaps if a
   node subscribes to the upstream names.
