================================================================================
APRILTAG DETECTION AND SINGLE FRAME PROCESSING
================================================================================

Author: Akhila Shaik
Date: December 16, 2025

================================================================================
1. APRILTAG ROS2 SETUP
================================================================================

INSTALLATION:
  sudo apt install ros-<distro>-apriltag-ros

CONFIGURATION (tags_36h11.yaml):
  /**:
    ros__parameters:
      tag_family: 36h11
      tag_sizes:
        0: 0.18    # Tag size in meters
        1: 0.18
        # Add all tag IDs

LAUNCH:
  ros2 run apriltag_ros apriltag_node \
    --ros-args \
    -r image_rect:=/camera2/camera2/color/image_raw \
    -r camera_info:=/camera2/camera2/color/camera_info \
    --params-file tags_36h11.yaml

VERIFY:
  ros2 topic echo /detections
  ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0

================================================================================
2. SINGLE FRAME PROCESSING (3 STEPS)
================================================================================

INSTALLATION:
  pip install numpy scipy opencv-python rerun-sdk rosbag2_py rclpy cv_bridge dt-apriltags

------------------------------------------------------------------------------
STEP 1: Extract all images from bag
------------------------------------------------------------------------------
Script: single_frame_apriltag_extraction.py

CONFIGURATION:
  BAG_PATH = "/path/to/rosbag/"
  STORAGE_ID = "mcap"  # or "sqlite3"
  IMAGE_TOPIC = "/camera2/camera2/color/image_raw"

RUN:
  python3 single_frame_apriltag_extraction.py

OUTPUT:
  - images/ folder with all frames
  - image_timestamps.json

NEXT: Browse images/ and pick one with clear AprilTag view (note the index)

------------------------------------------------------------------------------
STEP 2: Detect tags and sync LiDAR
------------------------------------------------------------------------------
Script: single_frame_apriltag_projection.py

CONFIGURATION:
  BAG_PATH = "/path/to/rosbag/"
  LIDAR_TOPIC = "/livox/lidar"
  TAG_SIZE = 0.18  # IMPORTANT: Your actual tag size in meters!

RUN:
  python3 single_frame_apriltag_projection.py <image_index>
  
EXAMPLE:
  python3 single_frame_apriltag_projection.py 1250

OUTPUT:
  - detections.json (tag poses in camera frame)
  - frame_pointcloud.npy (synchronized LiDAR)
  - selected_image_with_tags.jpg (visualization)
  - selected_frame_data.json (metadata)

NOTE: LiDAR extraction takes 5-10 minutes for large bags

------------------------------------------------------------------------------
STEP 3: Project to LiDAR frame and visualize
------------------------------------------------------------------------------
Script: visualize.py

CONFIGURATION (CRITICAL - UPDATE THIS!):
  # Camera-to-LiDAR calibration [x, y, z, qx, qy, qz, qw]
  T_LIDAR_CAMERA = [
    0.06223598068742114,
    0.012237596587866426,
    -0.05778298390566573,
    0.5035505589671723,
    0.4987164201169012,
    0.4961475523030442,
    0.5015539584265382
  ]

RUN:
  python3 visualize.py

OUTPUT: Rerun 3D viewer with:
  - Gray points = LiDAR pointcloud
  - Red spheres = AprilTag positions in LiDAR frame
  - RGB axes = Tag orientations
  - Yellow sphere = Camera position

================================================================================
3. COMPLETE WORKFLOW EXAMPLE
================================================================================

# Step 1: Extract images
python3 single_frame_apriltag_extraction.py
# Output: 27,395 images saved to images/

# Step 2: Pick good image (e.g., index 1250)
python3 single_frame_apriltag_projection.py 1250
# Output: Detected 3 tags, synced LiDAR

# Step 3: Visualize
python3 visualize.py
# Output: Rerun 3D viewer opens

================================================================================
4. TROUBLESHOOTING
================================================================================

NO TAGS DETECTED:
  - Pick different image with clearer tag view
  - Check TAG_FAMILY matches your tags
  - Verify lighting is good

WRONG PROJECTION IN STEP 3:
  - CHECK T_LIDAR_CAMERA calibration (most common!)
  - Verify TAG_SIZE is correct (measure your tags)
  - Ensure proper coordinate frame alignment

STEP 2 TAKES TOO LONG:
  - Normal for large MCAP bags (100GB+)
  - Script exits early if sync < 50ms

TOPIC NOT FOUND:
  - Check topics: ros2 bag info <bag_path>
  - Update topic names in scripts
  - Verify STORAGE_ID matches bag format


================================================================================
5. OUTPUT FILES
================================================================================

image_timestamps.json         - All images with timestamps
images/                       - Extracted frames folder
selected_frame_data.json      - Selected frame metadata
detections.json               - AprilTag poses (camera frame)
frame_pointcloud.npy          - Synchronized LiDAR points
selected_image_with_tags.jpg  - Visualization


================================================================================
REFERENCES
================================================================================

- AprilTag: https://april.eecs.umich.edu/software/apriltag
- ROS2 AprilTag: https://github.com/christianrauch/apriltag_ros
- Rerun: https://www.rerun.io/

