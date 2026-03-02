# Nav2 Wheelchair Implementation Documentation

## Overview
This document outlines the implementation, testing, and performance review of the Nav2 stack with the Wheelchair platform. It serves as a comprehensive guide for future development and testing.

## Setup Instructions

### Starting Nav2
1. Bringup Wheelchair
```bash
ros2 launch wheelchair2_bringup wheelchair2.launch.py
```
Set the `in_simulation` argument based on whether you are in simulation or not.

2. If running IRL, start LIDAR:

```bash
ros2 launch wheelchair2_bringup lidar.launch.py
```

3. Start Navigation2
```bash
ros2 launch wheelchair2_navigation nav2_amcl.launch.py
```
Set the `use_sim_time` argument based on whether you are in simulation or not.

### Configuration
The parameters are located in the `wheelchair2_navigation/config/nav2_params.yaml` file.

- **Global Planner**: NavfnPlanner (default)
  - Can be modified in the `planner_server` parameter
- **Local Planner**: DWB Planner
  - Can be modified in the `controller_server` parameter

## Testing Scenarios

### Basic Navigation Tests
1. **Open Space Navigation**
   - Environment: RRC Lab
   - Test Cases:
     - Straight line movement 
     - 90-degree turns
     - 180-degree turns

   - Environment: Other Environments will be added soon
   - Test Cases:
     - Straight line movement
     - 90-degree turns
     - 180-degree turns

2. **Obstacle Avoidance**
   - Static obstacles placed at:
     - Direct path
     - Corners
     - Narrow passages
   - Dynamic obstacles introduced during navigation

3. **Door Navigation**
   - Standard doorway 
   - Different approach angles

### Performance Testing
1. **Path Planning**
   - Time to generate global path
   - Path optimization
   - Replanning frequency
   - Close-Proximity approaches

2. **Motion Control**
   - Acceleration/deceleration smoothness
   - Turning radius accuracy
   - Speed control precision
   - Odometry Validation

3. **Localization**
   - Position tracking accuracy
   - Orientation accuracy

## Performance Review

### Strengths
1. **Reliable Path Planning**
   - Consistent global path generation
   - Efficient obstacle avoidance in most scenarios(need to tune inflation)
   - Good recovery behavior

2. **Motion Control**
   - Smooth acceleration/deceleration 
   - Stable motion in straight lines (no jerks in motion as of now)
   - Accurate stopping at goal positions 

### Areas for Improvement
1. **Navigation in Tight Spaces**
   - Occasional difficulty in narrow passages
   - Door frame navigation needs fine-tuning
   - Corner case handling needs improvement

2. **Dynamic Obstacle Handling**
   - Replanning delay with fast-moving obstacles
   - Conservative behavior in crowded spaces
   - Path oscillation in certain scenarios

## Failure Cases

### Critical Failures
1. **Localization Loss**
   - Causes:
     - Unknown obstacles density (too many people)
     - Sensor occlusion(2D LiDAR used, will switch to 3D)
     - Algorithmic limits
   - Recovery:
     - Tune AMCL Parameters
     - Add redundant sensor checks
     - Try other Localization methods (both for 2D and 3D)

2. **Path Planning Failures**
   - Causes:
     - Complex environment topology
     - Unreachable goals
     - Map inconsistencies (RRC Lab map is old, need to record new one once RTABMap Pipeline is ready)
   - Mitigation:
     - Implement timeout handling
     - Test other planners(CrowdSurfer? log-MPPI?)
     - Improve error reporting

## Future Tests

### Planned Test Scenarios

1. **Environmental Variations**
   - Different floor surfaces
   - Various lighting conditions
   - Multiple building layouts

2. **Integration Testing**
   - User interface integration (HMI Monitor?)
   - Safety system validation (What saftey features do we have except KILL switch?)

### Proposed Improvements
1. **Navigation Enhancement**
   - Custom recovery behaviors
   - Improved local planner tuning
   - Advanced obstacle prediction

2. **System Robustness**
   - Better error handling
   - Improved sensor fusion (Can we fuse different Odometry sources?)
   - Drift-Reduction (UMBmark paper?)

3. **User Experience**
   - Simplified configuration interface
   - Better progress monitoring
   - Improved status reporting

## Maintenance and Updates
- Regular parameter optimization
- Documentation updates
- Performance monitoring
- Bug tracking and resolution

## Contributing
Please submit improvements and bug reports through the issue tracker. For major changes, please open an issue first to discuss what you would like to change.

---
Last Updated: November 14, 2024