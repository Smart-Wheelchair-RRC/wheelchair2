# elevator_navigation

python-based multi-floor topological navigation and elevator state machine package for ros 2 humble on autonomous mobile robots and wheelchairs. coordinates transitions between floors, detects elevator vertical transit using livox imu vertical acceleration, verifies elevator door opening via 2d lidar scans, identifies destination floors using visual aruco markers or terminal override, and coordinates full localization/costmap memory wipes and reloads across dlio, 3d-bbs, icp localization, and nav2.

---

## package structure

```text
elevator_navigation/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── aruco_markers/
│   ├── aruco_markers.ipynb
│   └── marker_floor_0.png ... marker_floor_6.png
├── config/
│   └── campus_config.yaml
├── launch/
│   └── elevator_navigation.launch.py
├── shell_scripts/
│   ├── send_goal.sh
│   └── send_goal_4th_floor.sh
└── elevator_navigation/
    ├── __init__.py
    ├── elevator_floor_detection_aruco.py
    ├── elevator_floor_detection_manual.py
    ├── state_manager_node.py
    └── topological_router.py
```

---

## architecture & state flow

```text
               [send_goal.sh / user script]
                            │
               (/send_navigation_goal)
                            ▼
                ┌───────────────────────┐
                │  state_manager_node   │
                └───────────┬───────────┘
                            │
         ┌──────────────────┼──────────────────┐
         │                  │                  │
         ▼                  ▼                  ▼
  [nav2 action]     [pipeline memory]    [topological graph]
(navigate_to_pose)   (pause/wipe/load)      (networkx router)
                            ▲
                            │ (/detected_floor_id)
             ┌──────────────┴──────────────┐
             │ elevator_floor_detector     │
             │ (aruco or manual fallback)  │
             └─────────────────────────────┘
                            ▲
            ┌───────────────┴───────────────┐
      [/livox/imu]                      [/scan]
  (vertical jolt z)                (enclosure/doors)
```

1. **goal dispatch & route planning:** `send_goal.sh` publishes target building, floor, and pose to `/send_navigation_goal`. `state_manager_node` queries `TopologicalGraphRouter` to build a multi-stage plan.
2. **elevator transit detection:** when riding inside, `elevator_floor_detector` monitors `/livox/imu` linear acceleration along the z-axis. once resting gravity is calibrated, sustained vertical deviations over threshold confirm upward or downward motion.
3. **door opening & cabin exit:** front and rear rays from `/scan` confirm door opening. wheelchair forward motion tracked over `/odom` triggers the cabin exit sequence.
4. **floor confirmation & dispatch:** `elevator_floor_detection_aruco` locks onto the wall marker and immediately publishes the target floor to `/detected_floor_id`, keeping a background terminal prompt open for manual override.
5. **dynamic map reload & relocalization:** `state_manager_node` catches `/detected_floor_id`, pauses and wipes dlio, icp, and 3d-bbs buffers, generates updated runtime configs, reloads 2d and 3d maps, triggers 3d-bbs global localization via `/click_loc`, seeds initial pose, flushes nav2 costmaps, and resumes autonomous driving on the new floor.

---

## node descriptions

### 1. `state_manager_node.py`
the central state machine orchestrator:
- **lifecycle state machine:** tracks persistent robot states `IDLE`, `NAVIGATING`, `IN_ELEVATOR`, `PAUSED`.
- **service orchestrator:**
  - pause tracking: calls `/pause_odom`, `/pause_localization`, `/pause_3dbbs` via `std_srvs/srv/SetBool`.
  - memory wipe: calls `/reset_map`, `/reset_localization`, `/reset_3dbbs` via `std_srvs/srv/Empty`.
  - costmap flush: calls `/local_costmap/clear_entirely_local_costmap` via `nav2_msgs/srv/ClearEntireCostmap` to purge ghost artifacts.
- **runtime config generation:** dynamically writes `/tmp/current_floor_3dbbs.yaml` pointing to the destination floor pcd directory and updates node parameters on `/icp_localization` and `/gpu_ros2_test_rviz2`.
- **two-stage localization:** triggers 3d-bbs pose convergence over `/click_loc`, reloads nav2 map server via `/map_server/load_map`, and injects the calibrated elevator landing pose into `/initialpose`.
- **manual override recovery:** cancels running nav2 action goals if an override arrives on `/detected_floor_id` mid-transit.

### 2. `elevator_floor_detection_aruco.py`
automated marker-based floor identification node:
- **imu baseline calibration:** averages the first 100 vertical acceleration samples from `/livox/imu` to calibrate resting gravity.
- **motion validation:** triggers `IN_TRANSIT` only when vertical deviation $\vert{}a_z - g\vert{} > 0.020\text{ g}$ sustains for $> 0.35\text{ s}$ while the wheelchair is stationary ($v \le 0.05\text{ m/s}$) and inside cabin bounds.
- **enclosure check:** calculates 90th percentile of laser scan ranges to verify enclosed elevator walls ($r_{90} < 2.0\text{ m}$).
- **door opening detection:** checks median distance in forward and backward 30-degree cones against $1.35\text{ m}$.
- **marker decoding:** decodes `DICT_4X4_50` markers with error correction rate set to `0.6` to eliminate rotation aliasing. requires two consecutive frame agreements before locking the floor.
- **non-blocking exit handling:** publishes floor id immediately on drive-out ($\ge 0.45\text{ m}$) and frees the detector state back to `IDLE` while keeping an asynchronous override input prompt active in the terminal.

### 3. `elevator_floor_detection_manual.py`
manual terminal fallback node for environments without camera feed or aruco tags:
- identical imu vertical acceleration, lidar enclosure, and odometry exit tracking as the aruco node.
- immediately resets state to `IDLE` upon exiting the cabin so subsequent rides are not blocked.
- runs terminal prompt in a background daemon thread to accept floor integers `0-9` and publishes to `/detected_floor_id`.

### 4. `topological_router.py`
graph routing engine utilizing `networkx.DiGraph`:
- parses `campus_config.yaml` to register elevator approach and exit poses as nodes across floors.
- connects vertical shafts between matching elevator names with weighted directional edges.
- `get_route()` constructs transient `START` and `GOAL` nodes, computes dijkstra shortest paths, and outputs step-by-step navigation instructions: `navigate`, `wait_and_enter_elevator`, `pause_and_wipe_navigation`, and `reload_and_resume_navigation`.

---

## configuration guide: `config/campus_config.yaml`

the configuration file defines asset roots, base yaml templates, building layouts, map paths, and elevator coordinates.

### schema definition & rules

1. **`system` block:**
   - `base_dir`: absolute root directory path that gets prefixed to all relative map file paths.
   - `base_bbs_yaml`: absolute path to the template 3d-bbs configuration file copied and updated during runtime in `/tmp`.

2. **`buildings.<building_name>.floors.<floor_id>` hierarchy:**
   - `<building_name>`: unique string matching the building name passed in navigation goals, for example, `D-Block`.
   - `<floor_id>`: integer floor number, for example, `0`, `1`, `4`. must be an integer so it maps directly to `/detected_floor_id` and marker values.
   - `name`: human-readable identifier for logs, for example, `Ground_Floor`.
   - `maps.occupancy`: relative path from `base_dir` to the 2d costmap yaml loaded by `/map_server/load_map`.
   - `maps.pcd`: relative path from `base_dir` to the 3d point cloud file passed to `/icp_localization` and 3d-bbs.

3. **`elevators.<elevator_name>` hierarchy:**
   - `<elevator_name>`: unique shaft identifier, for example, `Left_Elevator`. `TopologicalGraphRouter` automatically connects floors that share the exact same elevator name.
   - `approach_pose`: `[x, y, yaw]` coordinate in meters and radians where the wheelchair stops to wait outside the elevator doors.
   - `exit_initial_pose`: `[x, y, yaw]` coordinate outside the doors on this floor. passed to `/initialpose` to seed localization immediately after the robot drives out of the cabin.

### example configuration

```yaml
system:
  base_dir: "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps"
  base_bbs_yaml: "/home/container_user/wheelchair2/src/dependencies/3DLocalization/wheelchair2_3dbbs/config/ros2_test.yaml"

buildings:
  D-Block:
    floors:
      0:
        name: "Ground_Floor"
        maps:
          occupancy: "ground_floor/rrc_lab.yaml"
          pcd: "ground_floor/lab_map_with_outside_corridor.pcd"
        elevators:
          Left_Elevator:
            approach_pose: [2.9641, 11.0742, 0.0115551]     # [x, y, yaw in radians]
            exit_initial_pose: [2.9641, 11.0742, 0.0115551] # [x, y, yaw in radians]

      4:
        name: "Fourth_Floor"
        maps:
          occupancy: "rrc/rrc_4th_floor/rrc_4th_floor_old.yaml"
          pcd: "rrc/rrc_4th_floor/rrc_4th_floor_old.pcd"
        elevators:
          Left_Elevator:
            approach_pose: [-6.13087, 18.142, 0.195863]
            exit_initial_pose: [-6.13087, 18.142, 0.195863]
```

---

## topics and interfaces

### published topics
- `/detected_floor_id` (`std_msgs/msg/Int32`): published by detector nodes on cabin exit.
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): seed pose sent to nav2 and icp.
- `/click_loc` (`std_msgs/msg/Bool`): trigger signal for 3d-bbs localization.

### subscribed topics
- `/livox/imu` (`sensor_msgs/msg/Imu`): vertical acceleration for transit detection.
- `/scan` (`sensor_msgs/msg/LaserScan`): lidar rays for enclosure and door state.
- `/odom` (`nav_msgs/msg/Odometry`): horizontal velocity and cabin exit displacement.
- `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`): camera feed for marker detection.
- `/send_navigation_goal` (`std_msgs/msg/String`): goal input in json format.

---

## build & run

### 1. build workspace
```bash
cd ~/wheelchair2
colcon build --packages-select elevator_navigation --symlink-install
source install/setup.bash
```

### 2. run state manager
```bash
ros2 run elevator_navigation state_manager_node
```

### 3. run detector
for automatic aruco detection:
```bash
ros2 run elevator_navigation elevator_floor_detection_aruco
```

OR

for manual terminal input:
```bash
ros2 run elevator_navigation elevator_floor_detection_manual
```

### 4. dispatch navigation goal
```bash
./src/elevator_navigation/shell_scripts/send_goal.sh Building_name Floor_id x y yaw
```
