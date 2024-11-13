# Instructions for running Nav2 with the Wheelchair Stack

## Starting Nav2
1. Bringup Wheelchair
```bash
ros2 launch wheelchair2_bringup wheelchair2.launch.py
```
Set the in_simulation argument based on whether you are in simulation or not.

2. Start Navigation2
```bash
ros2 launch wheelchair2_navigation nav2.launch.py
```
Set the in_simulation argument based on whether you are in simulation or not.

## Params
The parameters are located in the `wheelchair2_navigation/config/nav2_params.yaml` file.

- The global planner used by default is the NavfnPlanner. To change the planner, modify the `planner_server` parameter in the `nav2_params.yaml` file.
- The local planner used is the DWB Planner. To change the planner, modify the `controller_server` parameter in the `nav2_params.yaml` file.