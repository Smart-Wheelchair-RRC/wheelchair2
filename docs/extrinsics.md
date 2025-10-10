## Sensor Extrinsics 

The calibration results are in this format `T_lidar_camera: [x, y, z, qx, qy, qz, qw]` that transforms a 3D point in the camera frame into the LiDAR frame (i.e., `p_lidar = T_lidar_camera * p_camera`).

- **Transform between LiDAR and Camera1**(front facing realsense)
    ```json
    "T_lidar_camera1": [
      0.06223598068742114,
      0.012237596587866426,
      -0.05778298390566573,
      0.5035505589671723,
      0.4987164201169012,
      0.4961475523030442,
      0.5015539584265382
    ],
    ```