# #!/usr/bin/env python3
# """
# Process a specific frame by index number
# Extracts frame N, detects AprilTags, projects to map frame, visualizes in Rerun

# Usage: python process_specific_frame.py --frame-index 2

# Author: Akhila shaik
# Date: 2025-12-16
# """

# import rosbag2_py
# import numpy as np
# import cv2
# import argparse
# import json
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message
# from dt_apriltags import Detector
# from scipy.spatial.transform import Rotation as R
# import rerun as rr
# from cv_bridge import CvBridge

# # ============================================================
# # 🧩 CONFIGURATION
# # ============================================================

# BAG_PATH = "/media/akhila/Expansion/SARA/lab_markers/2025-10-17_01-31-14/rosbag/"
# STORAGE_ID = "mcap"

# # Topics
# IMAGE_TOPIC = "/camera2/camera2/color/image_raw"
# CAMERA_INFO_TOPIC = "/camera2/camera2/color/camera_info"
# LIDAR_TOPIC = "/livox/lidar"
# ODOMETRY_TOPIC = "/Odometry"
# LASER_MAP_TOPIC = "/Laser_map"

# # AprilTag settings
# TAG_FAMILY = "tag36h11"
# TAG_SIZE = 0.18  # meters

# # Extrinsic calibration: T_lidar_camera [x, y, z, qx, qy, qz, qw]
# T_LIDAR_CAMERA = [
#     0.06223598068742114,
#     0.012237596587866426,
#     -0.05778298390566573,
#     0.5035505589671723,
#     0.4987164201169012,
#     0.4961475523030442,
#     0.5015539584265382
# ]

# # Output files
# OUTPUT_DIR = "frame_output"
# OUTPUT_POSES = "poses.json"
# OUTPUT_TAG_IN_MAP = "tag_in_map.json"
# OUTPUT_IMAGE = "selected_frame_with_tags.jpg"

# # ============================================================
# # 🧮 Helper Functions
# # ============================================================

# def tum_to_matrix(extrinsics):
#     """Convert [x, y, z, qx, qy, qz, qw] → 4x4 transform matrix"""
#     x, y, z, qx, qy, qz, qw = extrinsics
#     rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
#     T = np.eye(4)
#     T[:3, :3] = rot
#     T[:3, 3] = [x, y, z]
#     return T

# def matrix_to_tum(T):
#     """Convert 4x4 transform matrix → [x, y, z, qx, qy, qz, qw]"""
#     pos = T[:3, 3]
#     quat = R.from_matrix(T[:3, :3]).as_quat()
#     return [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]

# def parse_pointcloud2(msg):
#     """Parse PointCloud2 message to numpy array"""
#     point_step = msg.point_step
#     num_points = msg.width * msg.height
#     points = []
    
#     for i in range(num_points):
#         offset = i * point_step
#         x = np.frombuffer(msg.data[offset:offset+4], dtype=np.float32)[0]
#         y = np.frombuffer(msg.data[offset+4:offset+8], dtype=np.float32)[0]
#         z = np.frombuffer(msg.data[offset+8:offset+12], dtype=np.float32)[0]
#         points.append([x, y, z])
    
#     return np.array(points)

# # ============================================================
# # 📥 Parse Arguments
# # ============================================================

# parser = argparse.ArgumentParser(description='Process specific frame by index')
# parser.add_argument('--frame-index', type=int, default=2,
#                     help='Frame index number to process (default: 2)')
# args = parser.parse_args()

# target_frame_index = args.frame_index

# # ============================================================
# # 📥 Open Rosbag
# # ============================================================

# print(f"🔍 Opening bag: {BAG_PATH}")
# print(f"🎯 Target frame index: {target_frame_index}")

# reader = rosbag2_py.SequentialReader()
# storage = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id=STORAGE_ID)
# converter = rosbag2_py.ConverterOptions('', '')
# reader.open(storage, converter)

# # Get message types
# topics = reader.get_all_topics_and_types()
# type_map = {t.name: t.type for t in topics}

# ImageType = get_message(type_map[IMAGE_TOPIC])
# CameraInfoType = get_message(type_map[CAMERA_INFO_TOPIC])
# PointCloud2Type = get_message(type_map[LIDAR_TOPIC])
# OdometryType = get_message(type_map[ODOMETRY_TOPIC])
# LaserMapType = get_message(type_map[LASER_MAP_TOPIC])

# bridge = CvBridge()

# # ============================================================
# # 📸 Extract Specific Frame
# # ============================================================

# print(f"\n📸 Finding frame {target_frame_index}...")

# camera_image = None
# camera_K = None
# camera_D = None
# target_timestamp = None
# frame_time = None

# current_frame_index = 0

# while reader.has_next():
#     topic, data, stamp = reader.read_next()
    
#     # Get camera info
#     if topic == CAMERA_INFO_TOPIC and camera_K is None:
#         msg = deserialize_message(data, CameraInfoType)
#         camera_K = np.array(msg.k).reshape(3, 3)
#         camera_D = np.array(msg.d)
    
#     # Find the target frame
#     if topic == IMAGE_TOPIC:
#         if current_frame_index == target_frame_index:
#             target_timestamp = stamp
#             msg = deserialize_message(data, ImageType)
#             camera_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
#             # Get bag start time for frame time calculation
#             metadata = reader.get_metadata()
#             start_time_ns = metadata.starting_time.nanoseconds
#             frame_time = (stamp - start_time_ns) / 1e9
            
#             print(f"✅ Found frame {target_frame_index}")
#             print(f"   Timestamp: {stamp} ns")
#             print(f"   Frame time: {frame_time:.3f} s from bag start")
#             print(f"   Image size: {camera_image.shape[1]}x{camera_image.shape[0]}")
#             break
        
#         current_frame_index += 1
        
#         if current_frame_index % 1000 == 0:
#             print(f"   Scanning... frame {current_frame_index}")

# if camera_image is None:
#     print(f"❌ Error: Frame {target_frame_index} not found!")
#     print(f"   Scanned up to frame {current_frame_index}")
#     exit(1)

# # ============================================================
# # 🏷️ Detect AprilTags
# # ============================================================

# print(f"\n🔍 Detecting AprilTags...")

# detector = Detector(families=TAG_FAMILY)
# gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)

# detections = detector.detect(
#     gray,
#     estimate_tag_pose=True,
#     camera_params=[camera_K[0,0], camera_K[1,1], camera_K[0,2], camera_K[1,2]],
#     tag_size=TAG_SIZE
# )

# print(f"✅ Detected {len(detections)} AprilTag(s)")

# if len(detections) == 0:
#     print("⚠️  No AprilTags detected in this frame!")
#     print("   Try a different frame index.")
#     exit(0)

# for det in detections:
#     print(f"   Tag ID {det.tag_id}")

# # Draw detections
# img_with_tags = camera_image.copy()
# for det in detections:
#     corners = det.corners.astype(np.int32)
#     cv2.polylines(img_with_tags, [corners], True, (0, 255, 0), 3)
#     center = corners.mean(axis=0).astype(np.int32)
#     cv2.putText(img_with_tags, f"ID: {det.tag_id}", 
#                 tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 
#                 1.0, (0, 255, 0), 2)

# # Save image
# import os
# os.makedirs(OUTPUT_DIR, exist_ok=True)
# output_image_path = os.path.join(OUTPUT_DIR, OUTPUT_IMAGE)
# cv2.imwrite(output_image_path, img_with_tags)
# print(f"💾 Saved image → {output_image_path}")

# # ============================================================
# # 📥 Extract Synchronized LiDAR and Odometry
# # ============================================================

# print(f"\n📥 Extracting synchronized data...")

# # Reset reader
# reader2 = rosbag2_py.SequentialReader()
# reader2.open(storage, converter)

# lidar_points = None
# odometry = None
# laser_map = None

# min_lidar_diff = float('inf')
# min_odom_diff = float('inf')

# lidar_timestamp = None
# odom_timestamp = None

# while reader2.has_next():
#     topic, data, stamp = reader2.read_next()
    
#     time_diff = abs(stamp - target_timestamp)
    
#     # Find closest LiDAR
#     if topic == LIDAR_TOPIC and time_diff < min_lidar_diff:
#         min_lidar_diff = time_diff
#         lidar_timestamp = stamp
#         msg = deserialize_message(data, PointCloud2Type)
#         lidar_points = parse_pointcloud2(msg)
    
#     # Find closest Odometry
#     if topic == ODOMETRY_TOPIC and time_diff < min_odom_diff:
#         min_odom_diff = time_diff
#         odom_timestamp = stamp
#         msg = deserialize_message(data, OdometryType)
#         odometry = {
#             'position': [msg.pose.pose.position.x, 
#                         msg.pose.pose.position.y, 
#                         msg.pose.pose.position.z],
#             'orientation': [msg.pose.pose.orientation.x,
#                            msg.pose.pose.orientation.y,
#                            msg.pose.pose.orientation.z,
#                            msg.pose.pose.orientation.w]
#         }
    
#     # Get Laser_map (just get the first one)
#     if topic == LASER_MAP_TOPIC and laser_map is None:
#         msg = deserialize_message(data, LaserMapType)
#         laser_map = parse_pointcloud2(msg)

# print(f"✅ Synchronized data:")
# print(f"   LiDAR: {len(lidar_points):,} points (diff: {min_lidar_diff/1e6:.2f} ms)")
# print(f"   Odometry: pos=[{odometry['position'][0]:.2f}, {odometry['position'][1]:.2f}, {odometry['position'][2]:.2f}] (diff: {min_odom_diff/1e6:.2f} ms)")
# if laser_map is not None:
#     print(f"   Laser_map: {len(laser_map):,} points")

# # ============================================================
# # 🔄 Project AprilTags to Map Frame
# # ============================================================

# print(f"\n🔄 Projecting AprilTags to Map frame...")
# print(f"   Transform chain: AprilTag → Camera → LiDAR → Map")

# # Build transforms
# T_lidar_camera = tum_to_matrix(T_LIDAR_CAMERA)

# T_map_lidar = np.eye(4)
# T_map_lidar[:3, 3] = odometry['position']
# T_map_lidar[:3, :3] = R.from_quat(odometry['orientation']).as_matrix()

# tag_projections = []

# for det in detections:
#     # T_camera_tag from detection
#     pose_t = det.pose_t.flatten()
#     pose_R = det.pose_R
    
#     T_camera_tag = np.eye(4)
#     T_camera_tag[:3, :3] = pose_R
#     T_camera_tag[:3, 3] = pose_t
    
#     # Full chain: T_map_tag = T_map_lidar @ T_lidar_camera @ T_camera_tag
#     T_map_tag = T_map_lidar @ T_lidar_camera @ T_camera_tag
    
#     tag_pos_map = T_map_tag[:3, 3]
#     tag_rot_map = T_map_tag[:3, :3]
    
#     tag_projections.append({
#         'id': int(det.tag_id),
#         'position_camera': pose_t.tolist(),
#         'position_map': tag_pos_map.tolist(),
#         'pose_map_tum': matrix_to_tum(T_map_tag),
#         'rotation_map': tag_rot_map
#     })
    
#     print(f"\n   Tag {det.tag_id}:")
#     print(f"      Camera: [{pose_t[0]:6.3f}, {pose_t[1]:6.3f}, {pose_t[2]:6.3f}] m")
#     print(f"      Map:    [{tag_pos_map[0]:6.3f}, {tag_pos_map[1]:6.3f}, {tag_pos_map[2]:6.3f}] m")

# # ============================================================
# # 💾 Save Outputs
# # ============================================================

# print(f"\n💾 Saving outputs...")

# # Save pose
# pose_data = {
#     'frame_index': target_frame_index,
#     'frame_time_sec': frame_time,
#     'timestamp_ns': int(target_timestamp),
#     'position': odometry['position'],
#     'orientation': odometry['orientation'],
#     'transform_tum': matrix_to_tum(T_map_lidar)
# }

# pose_path = os.path.join(OUTPUT_DIR, OUTPUT_POSES)
# with open(pose_path, 'w') as f:
#     json.dump(pose_data, f, indent=2)
# print(f"   ✅ {pose_path}")

# # Save tags
# tags_data = {
#     'frame_index': target_frame_index,
#     'frame_time_sec': frame_time,
#     'timestamp_ns': int(target_timestamp),
#     'num_tags': len(tag_projections),
#     'tags': tag_projections
# }

# tags_path = os.path.join(OUTPUT_DIR, OUTPUT_TAG_IN_MAP)
# with open(tags_path, 'w') as f:
#     json.dump(tags_data, f, indent=2)
# print(f"   ✅ {tags_path}")

# # ============================================================
# # 🎨 Visualize in Rerun
# # ============================================================

# print(f"\n🎨 Launching Rerun visualization...")
# rr.init("apriltag_map_projection", spawn=True)

# rr.set_time_seconds("timestamp", target_timestamp / 1e9)

# # Log Laser_map
# if laser_map is not None:
#     rr.log("map/laser_map", 
#            rr.Points3D(laser_map, radii=0.02, colors=[100, 100, 150]))

# # Log current LiDAR scan in map frame
# lidar_in_map = (T_map_lidar[:3, :3] @ lidar_points.T).T + T_map_lidar[:3, 3]
# rr.log("map/current_lidar_scan", 
#        rr.Points3D(lidar_in_map, radii=0.01, colors=[180, 180, 180]))

# # Log robot position
# lidar_pos_map = odometry['position']
# rr.log("map/robot_position", 
#        rr.Points3D([lidar_pos_map], radii=0.10, colors=[0, 255, 255]))

# # Log robot orientation axes
# axis_len = 0.5
# for axis_idx, (name, color) in enumerate([('x', [255, 100, 100]), 
#                                            ('y', [100, 255, 100]), 
#                                            ('z', [100, 100, 255])]):
#     axis_vec = np.zeros(3)
#     axis_vec[axis_idx] = axis_len
#     axis_end = lidar_pos_map + T_map_lidar[:3, :3] @ axis_vec
#     rr.log(f"map/robot/{name}_axis",
#            rr.LineStrips3D([[lidar_pos_map, axis_end]], colors=[color]))

# # Log camera position
# camera_pos_map = (T_map_lidar @ T_lidar_camera)[:3, 3]
# rr.log("map/camera_position", 
#        rr.Points3D([camera_pos_map], radii=0.08, colors=[255, 255, 0]))

# # Log camera image
# rr.log("camera/image", 
#        rr.Image(cv2.cvtColor(img_with_tags, cv2.COLOR_BGR2RGB)))

# # Log AprilTags in map frame
# for tag in tag_projections:
#     tag_id = tag['id']
#     pos = np.array(tag['position_map'])
#     rot = tag['rotation_map']
    
#     # Tag center
#     rr.log(f"map/apriltags/tag_{tag_id}/center",
#            rr.Points3D([pos], radii=0.10, colors=[255, 0, 0]))
    
#     # Tag axes
#     axis_length = 0.15
#     for axis_idx, (name, color) in enumerate([('x', [255, 0, 0]), 
#                                                ('y', [0, 255, 0]), 
#                                                ('z', [0, 0, 255])]):
#         axis_vec = np.zeros(3)
#         axis_vec[axis_idx] = axis_length
#         axis_end = pos + rot @ axis_vec
#         rr.log(f"map/apriltags/tag_{tag_id}/{name}_axis",
#                rr.LineStrips3D([[pos, axis_end]], colors=[color], radii=0.005))
    
#     # Label
#     rr.log(f"map/apriltags/tag_{tag_id}/label",
#            rr.TextDocument(f"Tag {tag_id}\nMap: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"))

# # ============================================================
# # 📊 Summary
# # ============================================================

# print(f"\n" + "="*70)
# print(f"🎯 FRAME {target_frame_index} COMPLETE!")
# print(f"="*70)

# print(f"\n📊 Summary:")
# print(f"   Frame: {target_frame_index} at {frame_time:.3f}s")
# print(f"   Detected tags: {len(detections)}")
# print(f"   Tag IDs: {[det.tag_id for det in detections]}")
# print(f"   Robot position: [{lidar_pos_map[0]:.2f}, {lidar_pos_map[1]:.2f}, {lidar_pos_map[2]:.2f}]")

# print(f"\n💾 Output files in '{OUTPUT_DIR}/':")
# print(f"   • {OUTPUT_IMAGE}")
# print(f"   • {OUTPUT_POSES}")
# print(f"   • {OUTPUT_TAG_IN_MAP}")

# print(f"\n🔍 Rerun Viewer:")
# print(f"   • Dark blue = Laser_map")
# print(f"   • Gray = Current LiDAR scan")
# print(f"   • Red = AprilTags in map frame")
# print(f"   • Cyan = Robot position")
# print(f"   • Yellow = Camera position")

# print("="*70)

"""
Verify AprilTag projection on static RRC map using ONE frame.
Saves the exact frame used for detection.

Pipeline:
AprilTag → Camera → LiDAR → Map (Odometry)

Usage:
python3 verify_apriltag_rrc_map_optimized.py --frame-index 2

Author: Akhila shaik
Date: 2025-12-16
Optimized: 2025-12-16
"""

import rosbag2_py
import numpy as np
import cv2
import argparse
import open3d as o3d
import rerun as rr

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge

# ============================================================
# CONFIG
# ============================================================

BAG_PATH = "/media/akhila/Expansion/SARA/lab_markers/2025-10-17_01-31-14/rosbag/"
STORAGE_ID = "mcap"

RRC_PCD_PATH = "/home/akhila/Downloads/RRC_april.pcd"

IMAGE_TOPIC = "/camera2/camera2/color/image_raw"
CAMERA_INFO_TOPIC = "/camera2/camera2/color/camera_info"
LIDAR_TOPIC = "/livox/lidar"
ODOMETRY_TOPIC = "/Odometry"

TAG_FAMILY = "tag36h11"
TAG_SIZE = 0.18  # meters

# T_lidar_camera [x,y,z,qx,qy,qz,qw]
T_LIDAR_CAMERA = [
    0.06223598068742114,
    0.012237596587866426,
    -0.05778298390566573,
    0.5035505589671723,
    0.4987164201169012,
    0.4961475523030442,
    0.5015539584265382
]

# Sync tolerance
MAX_SYNC_MS = 100  # Maximum allowed sync gap in milliseconds

# ============================================================
# HELPERS
# ============================================================

def tum_to_matrix(t):
    """Convert TUM format [x,y,z,qx,qy,qz,qw] to 4x4 transformation matrix."""
    x, y, z, qx, qy, qz, qw = t
    T = np.eye(4)
    T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def parse_pointcloud2(msg):
    """Parse PointCloud2 message to numpy array."""
    pts = []
    for i in range(msg.width * msg.height):
        o = i * msg.point_step
        x = np.frombuffer(msg.data[o:o+4], np.float32)[0]
        y = np.frombuffer(msg.data[o+4:o+8], np.float32)[0]
        z = np.frombuffer(msg.data[o+8:o+12], np.float32)[0]
        pts.append([x, y, z])
    return np.asarray(pts)


def create_reader(bag_path, storage_id):
    """Create a new rosbag reader."""
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, converter)
    return reader


# ============================================================
# ARGUMENTS
# ============================================================

parser = argparse.ArgumentParser(description="Verify AprilTag projection on RRC map")
parser.add_argument("--frame-index", type=int, default=2, help="Image frame index to use")
args = parser.parse_args()

# ============================================================
# LOAD STATIC MAP
# ============================================================

print("📥 Loading RRC map...")
pcd = o3d.io.read_point_cloud(RRC_PCD_PATH)
map_points = np.asarray(pcd.points)
print(f"✅ Map points: {len(map_points):,}")

# ============================================================
# OPEN ROSBAG
# ============================================================

print("📂 Opening rosbag...")
reader = create_reader(BAG_PATH, STORAGE_ID)

types = {t.name: t.type for t in reader.get_all_topics_and_types()}

ImageT = get_message(types[IMAGE_TOPIC])
CameraInfoT = get_message(types[CAMERA_INFO_TOPIC])
PointCloudT = get_message(types[LIDAR_TOPIC])
OdomT = get_message(types[ODOMETRY_TOPIC])

bridge = CvBridge()

# ============================================================
# EXTRACT IMAGE FRAME
# ============================================================

print(f"📸 Extracting image frame index {args.frame_index}...")

camera_img = None
camera_K = None
target_ts = None
idx = 0

while reader.has_next():
    topic, data, ts = reader.read_next()

    if topic == CAMERA_INFO_TOPIC and camera_K is None:
        msg = deserialize_message(data, CameraInfoT)
        camera_K = np.array(msg.k).reshape(3, 3)

    if topic == IMAGE_TOPIC:
        if idx == args.frame_index:
            msg = deserialize_message(data, ImageT)
            camera_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            target_ts = ts
            print(f"✅ Found frame at timestamp: {target_ts}")
            break
        idx += 1

# Validate
if camera_img is None:
    raise RuntimeError(f"❌ Frame index {args.frame_index} not found in bag")

if camera_K is None:
    raise RuntimeError("❌ Camera intrinsics not found")

# Save image used
cv2.imwrite("used_frame.jpg", camera_img)
print("✅ Saved image → used_frame.jpg")

# ============================================================
# APRILTAG DETECTION
# ============================================================

print("🔍 Detecting AprilTags...")
detector = Detector(families=TAG_FAMILY)
gray = cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY)

detections = detector.detect(
    gray,
    estimate_tag_pose=True,
    camera_params=[camera_K[0, 0], camera_K[1, 1], camera_K[0, 2], camera_K[1, 2]],
    tag_size=TAG_SIZE
)

if len(detections) == 0:
    raise RuntimeError("❌ No AprilTags detected in frame")

print(f"✅ Detected {len(detections)} tag(s): {[d.tag_id for d in detections]}")

# ============================================================
# SYNC LIDAR + ODOM (OPTIMIZED)
# ============================================================

print("⏳ Syncing LiDAR and Odometry...")
reader2 = create_reader(BAG_PATH, STORAGE_ID)

lidar_raw = None
odom_raw = None
min_lidar = 1e18
min_odom = 1e18
msg_count = 0

while reader2.has_next():
    topic, data, ts = reader2.read_next()
    dt = abs(ts - target_ts)

    # Store raw bytes only — don't parse yet
    if topic == LIDAR_TOPIC and dt < min_lidar:
        lidar_raw = data
        min_lidar = dt

    if topic == ODOMETRY_TOPIC and dt < min_odom:
        odom_raw = data
        min_odom = dt

    # Progress indicator
    msg_count += 1
    if msg_count % 10000 == 0:
        print(f"   Processed {msg_count:,} messages...")

    # Early exit: if we've passed target by 2 seconds and have both, stop
    if ts > target_ts + 2e9 and lidar_raw is not None and odom_raw is not None:
        print("   ✓ Early exit - found sync data")
        break

# Validate
if lidar_raw is None:
    raise RuntimeError("❌ No LiDAR data found near target timestamp")

if odom_raw is None:
    raise RuntimeError("❌ No Odometry data found near target timestamp")

# Parse only the best matches
print("📦 Parsing synced messages...")
lidar = parse_pointcloud2(deserialize_message(lidar_raw, PointCloudT))
odom = deserialize_message(odom_raw, OdomT)

print(f"✅ LiDAR Δt: {min_lidar/1e6:.2f} ms ({len(lidar):,} points)")
print(f"✅ Odom  Δt: {min_odom/1e6:.2f} ms")

# Warn if sync gap is large
if min_lidar / 1e6 > MAX_SYNC_MS:
    print(f"⚠️  Warning: LiDAR sync gap ({min_lidar/1e6:.1f} ms) exceeds {MAX_SYNC_MS} ms")
if min_odom / 1e6 > MAX_SYNC_MS:
    print(f"⚠️  Warning: Odom sync gap ({min_odom/1e6:.1f} ms) exceeds {MAX_SYNC_MS} ms")

# ============================================================
# TRANSFORMS
# ============================================================

print("🔄 Computing transforms...")

T_lidar_camera = tum_to_matrix(T_LIDAR_CAMERA)

T_map_lidar = np.eye(4)
T_map_lidar[:3, 3] = [
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z
]
T_map_lidar[:3, :3] = R.from_quat([
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w
]).as_matrix()

print(f"   Robot position: [{odom.pose.pose.position.x:.3f}, "
      f"{odom.pose.pose.position.y:.3f}, {odom.pose.pose.position.z:.3f}]")

# ============================================================
# PROJECT TAGS TO MAP
# ============================================================

print("🏷️  Projecting AprilTags to map frame...")

tag_positions = []

for d in detections:
    # Tag pose in camera frame
    T_cam_tag = np.eye(4)
    T_cam_tag[:3, :3] = d.pose_R
    T_cam_tag[:3, 3] = d.pose_t.flatten()

    # Transform to map frame
    T_map_tag = T_map_lidar @ T_lidar_camera @ T_cam_tag
    pos = T_map_tag[:3, 3]
    tag_positions.append((d.tag_id, pos, T_map_tag))

    print(f"   Tag {d.tag_id}: Map position = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

# ============================================================
# RERUN VISUALIZATION
# ============================================================

print("🎨 Launching Rerun...")
rr.init("apriltag_rrc_map_check", spawn=True)

# Log static map
rr.log("map/rrc", rr.Points3D(map_points, radii=0.02, colors=[80, 80, 120]))

# Log LiDAR transformed to map frame
lidar_map = (T_map_lidar[:3, :3] @ lidar.T).T + T_map_lidar[:3, 3]
rr.log("map/lidar", rr.Points3D(lidar_map, radii=0.01, colors=[0, 200, 0]))

# Log robot position
rr.log("map/robot", rr.Points3D([T_map_lidar[:3, 3]], radii=0.15, colors=[0, 255, 255]))

# Log AprilTag positions
for tid, pos, T_map_tag in tag_positions:
    rr.log(f"map/apriltag_{tid}",
           rr.Points3D([pos], radii=0.15, colors=[255, 0, 0]))
    
    # Log tag coordinate frame
    rr.log(f"map/apriltag_{tid}/frame",
           rr.Transform3D(
               translation=pos,
               mat3x3=T_map_tag[:3, :3]
           ))

# Log camera image with detections drawn
img_annotated = camera_img.copy()
for d in detections:
    # Draw tag outline
    corners = d.corners.astype(int)
    cv2.polylines(img_annotated, [corners], True, (0, 255, 0), 2)
    # Draw center
    cx, cy = int(d.center[0]), int(d.center[1])
    cv2.circle(img_annotated, (cx, cy), 5, (0, 0, 255), -1)
    # Draw tag ID
    cv2.putText(img_annotated, f"ID:{d.tag_id}", (cx + 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

rr.log("camera/image", rr.Image(cv2.cvtColor(img_annotated, cv2.COLOR_BGR2RGB)))

# Save annotated image
cv2.imwrite("used_frame_annotated.jpg", img_annotated)
print("✅ Saved annotated image → used_frame_annotated.jpg")

# ============================================================
# SUMMARY
# ============================================================

print("\n" + "=" * 60)
print("✅ DONE")
print("=" * 60)
print(f"📸 Frame used: used_frame.jpg")
print(f"📸 Annotated:  used_frame_annotated.jpg")
print(f"🏷️  Tags found: {len(tag_positions)}")
for tid, pos, _ in tag_positions:
    print(f"   - Tag {tid}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
print("👀 Verify alignment in Rerun viewer")
print("=" * 60)
