APRILTAG POSITION MAPPING PIPELINE

This pipeline processes ROS2 bag files to create a precise map of AprilTag 
positions in the global coordinate frame. It extracts tag detections, 
synchronizes them with robot poses, filters based on distance thresholds, 
removes outliers, and computes averaged positions for each tag.

PIPELINE WORKFLOW:
ROS2 Bag Files
      |
      v
[1] detections.py          -> detections.json
[2] extract_odometry_poses.py -> poses.json
[3] image_timestamps.py    -> image_timestamps.json
      |
      v
[4] sync_image_pose.py     -> pose_image_sync.json
      |
      v
[5] compute_based_on_distance.py -> distance3_tags.json + .ply
      |
      v
[6] visualisation.py       -> 3D Viewer


INSTALLATION

PREREQUISITES:
# ROS2 (Humble/Iron/Rolling)
sudo apt install ros-<distro>-rosbag2 ros-<distro>-cv-bridge

# Python packages
pip install numpy scipy opencv-python rerun-sdk rosbag2_py rclpy



SCRIPTS DESCRIPTION

1. detections.py
Extracts AprilTag 3D transforms from /tf topic.

INPUT: ROS2 bag with /tf topic
OUTPUT: detections.json
RUN:python3 detections.py

2. extract_odometry_poses.py
Extracts robot odometry from /Odometry topic.

INPUT: ROS2 bag with /Odometry topic
OUTPUT: poses.json
RUN:python3 extract_odometry_poses.py


3. image_timestamps.py
Extracts camera images and timestamps.

INPUT: ROS2 bag with camera topic
OUTPUT: images/ folder + image_timestamps.json
RUN:python3 image_timestamps.py


4. sync_image_pose.py
Synchronizes image timestamps with odometry poses.

INPUT: poses.json + image_timestamps.json
OUTPUT: pose_image_sync.json
RUN:python3 sync_image_pose.py

5. compute_based_on_distance.py (MAIN SCRIPT)
Computes final tag positions with filtering and outlier removal.

INPUT:
- pose_image_sync.json
- detections.json
- Camera-LiDAR calibration parameters

OUTPUT:
- distance3_tags.json (with statistics)
- distance3_tags.ply (point cloud)
RUN:python3 compute_based_on_distance.py


6. visualisation.py
Interactive 3D visualization using Rerun.

INPUT:
- RRC_april.pcd (map point cloud)
- distance3_tags.json (tag positions)
RUN:
  python3 visualisation.py



COORDINATE FRAMES

map (global reference frame)
 |
 +-- odom (robot base_link / LiDAR frame)
      |
      +-- camera2 (camera optical frame)
           |
           +-- tag36h11:X (AprilTag detections)

TRANSFORM CHAIN:
  T_map_tag = T_map_odom @ T_odom_camera @ T_camera_tag



REFERENCES
================================================================================

- AprilTag Library: https://april.eecs.umich.edu/software/apriltag
- ROS2 Documentation: https://docs.ros.org/en/rolling/
- Rerun Visualization: https://www.rerun.io/





