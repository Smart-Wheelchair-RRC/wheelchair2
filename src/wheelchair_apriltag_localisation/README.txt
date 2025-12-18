================================================================================
APRILTAG POSITION MAPPING PIPELINE
================================================================================

Author: Akhila Shaik
Date: November 12, 2025
Purpose: Extract AprilTag detections from ROS2 bags, synchronize with robot 
         odometry, and compute accurate 3D positions in the map frame.

================================================================================
OVERVIEW
================================================================================

This pipeline processes ROS2 bag files to create a precise map of AprilTag 
positions in the global coordinate frame. It extracts tag detections, 
synchronizes them with robot poses, filters based on distance thresholds, 
removes outliers, and computes averaged positions for each tag.

KEY FEATURES:
- Distance-based filtering (only uses detections within 3 meters)
- Robust outlier removal with iterative statistical filtering
- Time-synchronized pose matching
- Quality metrics and uncertainty quantification
- 3D visualization support

================================================================================
PIPELINE WORKFLOW
================================================================================

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

================================================================================
INSTALLATION
================================================================================

PREREQUISITES:

# ROS2 (Humble/Iron/Rolling)
sudo apt install ros-<distro>-rosbag2 ros-<distro>-cv-bridge

# Python packages
pip install numpy scipy opencv-python rerun-sdk rosbag2_py rclpy

================================================================================
SCRIPTS DESCRIPTION
================================================================================

------------------------------------------------------------------------------
1. detections.py
------------------------------------------------------------------------------
Extracts AprilTag 3D transforms from /tf topic.

INPUT: ROS2 bag with /tf topic
OUTPUT: detections.json

CONFIGURATION:
  BAG_PATH = "/home/akhila/apriltag_synced_bag_fresh/"
  TF_TOPIC = "/tf"
  OUTPUT_JSON = "detections.json"

RUN:
  python3 detections.py

------------------------------------------------------------------------------
2. extract_odometry_poses.py
------------------------------------------------------------------------------
Extracts robot odometry from /Odometry topic.

INPUT: ROS2 bag with /Odometry topic
OUTPUT: poses.json

CONFIGURATION:
  bag_path = "/media/akhila/Expansion/SARA/lab_markers/2025-10-17_01-31-14/rosbag"
  odom_topic = "/Odometry"

RUN:
  python3 extract_odometry_poses.py

------------------------------------------------------------------------------
3. image_timestamps.py
------------------------------------------------------------------------------
Extracts camera images and timestamps.

INPUT: ROS2 bag with camera topic
OUTPUT: images/ folder + image_timestamps.json

CONFIGURATION:
  bag_path = "/home/akhila/apriltag_synced_bag_fresh"
  image_topic = "/camera2/camera2/color/image_raw"
  output_dir = "images"

RUN:
  python3 image_timestamps.py

------------------------------------------------------------------------------
4. sync_image_pose.py
------------------------------------------------------------------------------
Synchronizes image timestamps with odometry poses.

INPUT: poses.json + image_timestamps.json
OUTPUT: pose_image_sync.json

FEATURES:
- Temporal nearest-neighbor matching
- Handles timestamp format conversion
- Reports sync quality statistics

RUN:
  python3 sync_image_pose.py

------------------------------------------------------------------------------
5. compute_based_on_distance.py (MAIN SCRIPT)
------------------------------------------------------------------------------
Computes final tag positions with filtering and outlier removal.

INPUT:
- pose_image_sync.json
- detections.json
- Camera-LiDAR calibration parameters

OUTPUT:
- distance3_tags.json (with statistics)
- distance3_tags.ply (point cloud)

KEY PARAMETERS:
  MAX_DISTANCE = 3          # Only use detections within 3 meters
  MAX_TIME_DIFF = 0.05      # 50ms time sync tolerance
  MIN_DETECTIONS = 5        # Minimum observations per tag
  OUTLIER_THRESHOLD = 2.5   # Statistical outlier removal (sigma)
  MAX_ITERATIONS = 5        # Iterative filtering passes

  # Camera-to-LiDAR calibration (x,y,z,qx,qy,qz,qw)
  T_LIDAR_CAMERA = [0.0622, 0.0122, -0.0578, 0.5036, 0.4987, 0.4961, 0.5016]

QUALITY METRICS:
- Excellent: std < 0.15m
- Good: 0.15m <= std < 0.3m
- Poor: std >= 0.3m

RUN:
  python3 compute_based_on_distance.py

------------------------------------------------------------------------------
6. visualisation.py
------------------------------------------------------------------------------
Interactive 3D visualization using Rerun.

INPUT:
- RRC_april.pcd (map point cloud)
- distance3_tags.json (tag positions)

FEATURES:
- Map displayed in gray
- Tags shown as colored 3D boxes
- Labels with ID and detection count
- Supports ASCII and binary PCD formats

RUN:
  python3 visualisation.py

CONTROLS:
- Left Mouse -> Rotate
- Right Mouse -> Pan
- Scroll -> Zoom
- Press F -> Focus

================================================================================
QUICK START
================================================================================

STEP-BY-STEP EXECUTION:

# Step 1: Extract all raw data
python3 detections.py
python3 extract_odometry_poses.py
python3 image_timestamps.py

# Step 2: Synchronize
python3 sync_image_pose.py

# Step 3: Compute final positions
python3 compute_based_on_distance.py

# Step 4: Visualize
python3 visualisation.py

================================================================================
OUTPUT FILES
================================================================================

FILE                        DESCRIPTION
--------------------------------------------------------------------------------
detections.json             Raw tag detections in camera frame
poses.json                  Robot odometry poses
image_timestamps.json       Camera frame timestamps
pose_image_sync.json        Synchronized poses
distance3_tags.json         Final tag positions + statistics
distance3_tags.ply          Tag positions (PLY format)

================================================================================
CONFIGURATION
================================================================================

UPDATE BAG PATHS:
Edit paths in each script:

  # detections.py
  BAG_PATH = "/your/path/to/bag/"

  # extract_odometry_poses.py
  bag_path = "/your/path/to/bag/"

  # image_timestamps.py
  bag_path = "/your/path/to/bag/"

  # compute_based_on_distance.py
  POSES_JSON = "/your/path/to/pose_image_sync.json"
  DETECTIONS_JSON = "/your/path/to/detections.json"

UPDATE CALIBRATION:
In compute_based_on_distance.py:
  T_LIDAR_CAMERA = [x, y, z, qx, qy, qz, qw]  # Your calibration

UPDATE STORAGE FORMAT:
If using MCAP bags instead of SQLite3:
  storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")

================================================================================
COORDINATE FRAMES
================================================================================

map (global reference frame)
 |
 +-- odom (robot base_link / LiDAR frame)
      |
      +-- camera2 (camera optical frame)
           |
           +-- tag36h11:X (AprilTag detections)

TRANSFORM CHAIN:
  T_map_tag = T_map_odom @ T_odom_camera @ T_camera_tag

================================================================================
TROUBLESHOOTING
================================================================================

PROBLEM: No tags detected
SOLUTION:
- Check if /tf topic exists: ros2 bag info <bag_path>
- Verify frame names match filter in detections.py
- Ensure AprilTag detector was running during recording

PROBLEM: High position uncertainty
SOLUTION:
- Increase MIN_DETECTIONS (require more observations)
- Decrease MAX_DISTANCE (use only close detections)
- Verify camera-LiDAR calibration accuracy

PROBLEM: Time synchronization errors
SOLUTION:
- Check timestamp formats (nanoseconds vs seconds)
- Increase MAX_TIME_DIFF tolerance
- Verify both topics were recorded simultaneously

PROBLEM: Visualization doesn't load
SOLUTION:
- Install Rerun: pip install rerun-sdk
- Check PCD file path and format
- Verify JSON file encoding (use UTF-8)

================================================================================
TECHNICAL DETAILS
================================================================================

DISTANCE FILTERING RATIONALE:
- AprilTag detection accuracy degrades with distance
- Close-range detections (<=3m) provide most reliable measurements
- Filters out ~60-70% of detections but dramatically improves precision

OUTLIER REMOVAL ALGORITHM:
1. Compute mean position from all detections
2. Calculate distances from mean
3. Compute standard deviation
4. Remove points > 2.5 sigma from mean
5. Repeat until convergence or max iterations

QUALITY METRICS:
- mean_std: Average standard deviation across x, y, z
- count: Number of inlier detections used
- avg_distance: Mean detection distance from camera

================================================================================
REFERENCES
================================================================================

- AprilTag Library: https://april.eecs.umich.edu/software/apriltag
- ROS2 Documentation: https://docs.ros.org/en/rolling/
- Rerun Visualization: https://www.rerun.io/





