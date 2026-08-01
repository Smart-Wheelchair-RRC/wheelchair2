# Nav2 Params Migration Guide: Wheelchair2 to Silica Demo

This guide documents the exact changes needed to migrate the Nav2 parameters file in the `wheelchair2` repository to match the `wc2_silica_demo` configuration.

**Target File**: `/home/robot/wheelchair_ws/wheelchair2/wheelchair2_navigation/config/nav2_params.yaml`

---

## Change Checklist

1. [Update Transform Tolerance Parameters](#1-update-transform-tolerance-parameters)
2. [Update Local Costmap Footprint](#2-update-local-costmap-footprint)
3. [Update Local Costmap Observation Sources](#3-update-local-costmap-observation-sources)
4. [Insert Camera & Glass Detection Sensor Layers](#4-insert-camera--glass-detection-sensor-layers)

---

### 1. Update Transform Tolerance Parameters

To reduce latency tolerances for coordinate transformations back to the standard demo settings, update the following values from `1.0` to `0.3` (or `0.2`):

#### A. under `bt_navigator` (Line 114)
```yaml
# Find this block:
bt_navigator:
  ros__parameters:
    ...
    robot_base_frame: base_footprint
-   transform_tolerance: 1.0
+   transform_tolerance: 0.3
```

#### B. under `controller_server` (Line 188)
```yaml
# Find this block:
controller_server:
  ros__parameters:
    ...
    failure_tolerance: 1.0
-   transform_tolerance: 1.0
+   transform_tolerance: 0.3
```

#### C. under `FollowPath` controller configuration (Line 227)
```yaml
# Find this block:
    FollowPath:
      ...
      prune_distance: 1.7
-     transform_tolerance: 1.0
+     transform_tolerance: 0.3
```

#### D. under `local_costmap` obstacle layer (Line 358)
```yaml
# Find this block:
      lidar_obstacle_layer:
        ...
        publish_voxel_map: false # default off
-       transform_tolerance: 1.0 # seconds
+       transform_tolerance: 0.2 # seconds
```

#### E. under `collision_monitor` (Line 609)
```yaml
# Find this block:
collision_monitor:
  ros__parameters:
    ...
    state_topic: "collision_monitor_state"
-   transform_tolerance: 1.0
+   transform_tolerance: 0.2
```

---

### 2. Update Local Costmap Footprint

Update the local costmap footprint to match the demo polygon.

#### A. under `local_costmap` (Line 332)
```yaml
# Replace this line:
-     footprint: "[ [0.78, 0.33], [0.78, -0.33], [-0.35, -0.33], [-0.35, 0.33] ]"
+     footprint: "[[-0.30,-0.32],[0.72,-0.32],[0.72,0.32],[-0.30,0.32]]"
```

---

### 3. Update Local Costmap Observation Sources

Append camera and glass detection sources to the local costmap's observation sources.

#### A. under `local_costmap` → `lidar_obstacle_layer` (Line 361)
```yaml
# Replace this line:
-       observation_sources: lidar1_mark lidar1_clear
+       observation_sources: lidar1_mark lidar1_clear glass_mark glass_clear rs_mark rs_clear
```

---

### 4. Insert Camera & Glass Detection Sensor Layers

Insert the definitions for `rs_mark`, `rs_clear`, `glass_mark`, and `glass_clear` right after `lidar1_clear` configuration block (directly **before** `inflation_layer:` at line 392).

#### A. Insertion point (Line 391)
Paste the following block immediately before `inflation_layer:`:

```yaml
        rs_mark:
          data_type: PointCloud2
          topic: /camera1/camera1/depth/color/points # Replace with your PointCloud2 topic
          marking: true
          clearing: false
          obstacle_range: 5.0
          min_obstacle_height: 0.5
          max_obstacle_height: 2.0
          expected_update_rate: 0.0
          observation_persistence: 0.0
          inf_is_valid: false
          filter: "voxel"
          voxel_min_points: 0
          clear_after_reading: true
          model_type: 0 # 0=depth camera

        rs_clear:
          enabled: true
          data_type: PointCloud2
          topic: /camera1/camera1/depth/color/points # Replace with your PointCloud2 topic
          marking: false
          clearing: true
          max_z: 5.0
          min_z: 0.05
          vertical_fov_angle: 1.01
          vertical_fov_padding: 0.0
          horizontal_fov_angle: 1.51
          decay_acceleration: 5.0
          model_type: 0

        glass_mark:
          data_type: PointCloud2
          topic: /glass_aware/depth/color/points # Replace with your PointCloud2 topic
          marking: true
          clearing: false
          obstacle_range: 5.0
          min_obstacle_height: 0.5
          max_obstacle_height: 2.0
          expected_update_rate: 0.0
          observation_persistence: 0.0
          inf_is_valid: false
          filter: "voxel"
          voxel_min_points: 0
          clear_after_reading: false
          model_type: 0 # 0=depth camera

        glass_clear:
          enabled: true
          data_type: PointCloud2
          topic: /glass_aware/depth/color/points # Replace with your PointCloud2 topic
          marking: false
          clearing: true
          max_z: 5.0
          min_z: 0.05
          vertical_fov_angle: 1.01
          vertical_fov_padding: 0.0
          horizontal_fov_angle: 1.51
          decay_acceleration: 5.0
          model_type: 0
```
