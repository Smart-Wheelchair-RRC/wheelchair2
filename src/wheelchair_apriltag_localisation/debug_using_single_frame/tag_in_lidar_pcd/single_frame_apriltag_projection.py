#!/usr/bin/env python3
"""
Step 2 for MCAP bag: Process a selected image - detect AprilTags, extract synchronized LiDAR
Usage: python mcap_step2_process_selected_image.py <image_index>
Example: python mcap_step2_process_selected_image.py 1

Author: Akhila Shaik
Date: 2025-12-16
"""

import rosbag2_py
import json
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import cv2
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation as R

# ============================================================
# 🧩 CONFIGURATION
# ============================================================

BAG_PATH = "/media/akhila/Expansion/SARA/lab_markers/2025-10-17_01-31-14/rosbag/"
STORAGE_ID = "mcap"
LIDAR_TOPIC = "/livox/lidar"
CAMERA_INFO_TOPIC = "/camera2/camera2/color/camera_info"

IMAGE_DIR = "images"
TIMESTAMPS_FILE = "image_timestamps.json"

# AprilTag parameters
TAG_FAMILY = "tag36h11"
TAG_SIZE = 0.18  # meters

# Output files
OUTPUT_FRAME_DATA = "selected_frame_data.json"
OUTPUT_DETECTIONS = "detections.json"
OUTPUT_POINTCLOUD = "frame_pointcloud.npy"
OUTPUT_IMAGE = "selected_image.jpg"
OUTPUT_IMAGE_WITH_TAGS = "selected_image_with_tags.jpg"

# ============================================================
# 📥 Get Selected Image Index
# ============================================================

if len(sys.argv) < 2:
    print("❌ Usage: python mcap_step2_process_selected_image.py <image_index>")
    print("   Example: python mcap_step2_process_selected_image.py 1")
    sys.exit(1)

selected_index = int(sys.argv[1])

# ============================================================
# 📥 Load Image Timestamps
# ============================================================

print(f"📥 Loading image timestamps...")
with open(TIMESTAMPS_FILE, "r") as f:
    image_data = json.load(f)

# Find selected image
selected_image_info = None
for img_info in image_data:
    if img_info["index"] == selected_index:
        selected_image_info = img_info
        break

if selected_image_info is None:
    print(f"❌ Image index {selected_index} not found!")
    print(f"   Valid range: 0 to {len(image_data)-1}")
    sys.exit(1)

target_timestamp = selected_image_info["timestamp_ns"]
image_filename = selected_image_info["filename"]

print(f"✅ Selected image: {image_filename}")
print(f"   Timestamp: {target_timestamp} ns ({target_timestamp/1e9:.3f} s)")

# ============================================================
# 🏷️ Detect AprilTags in Selected Image
# ============================================================

print("🔍 Detecting AprilTags...")

# Load the image
image_path = f"{IMAGE_DIR}/{image_filename}"
camera_image = cv2.imread(image_path)

if camera_image is None:
    print(f"❌ Could not load image: {image_path}")
    sys.exit(1)

# Save copy of selected image
cv2.imwrite(OUTPUT_IMAGE, camera_image)
print(f"💾 Saved selected image → {OUTPUT_IMAGE}")

# Get camera info from bag
print("📥 Extracting camera info from bag...")
reader = rosbag2_py.SequentialReader()
storage = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id=STORAGE_ID)
converter = rosbag2_py.ConverterOptions('', '')
reader.open(storage, converter)

topics = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topics}

CameraInfoType = get_message(type_map[CAMERA_INFO_TOPIC])

camera_K = None
camera_D = None

while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == CAMERA_INFO_TOPIC:
        msg = deserialize_message(data, CameraInfoType)
        camera_K = np.array(msg.k).reshape(3, 3)
        camera_D = np.array(msg.d)
        break

print(f"✅ Got camera intrinsics")

# Run AprilTag detection
detector = Detector(families=TAG_FAMILY)
gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)

detections = detector.detect(
    gray,
    estimate_tag_pose=True,
    camera_params=[camera_K[0,0], camera_K[1,1], camera_K[0,2], camera_K[1,2]],
    tag_size=TAG_SIZE
)

print(f"✅ Detected {len(detections)} AprilTag(s)")

if len(detections) == 0:
    print("⚠️  No AprilTags detected! Choose a different image.")
    sys.exit(0)

# Save detections in camera frame
detections_list = []
for det in detections:
    pose_t = det.pose_t.flatten()
    pose_R = det.pose_R
    
    # Convert rotation matrix to quaternion
    quat = R.from_matrix(pose_R).as_quat()  # [qx, qy, qz, qw]
    
    detections_list.append({
        "id": int(det.tag_id),
        "tx": float(pose_t[0]),
        "ty": float(pose_t[1]),
        "tz": float(pose_t[2]),
        "qx": float(quat[0]),
        "qy": float(quat[1]),
        "qz": float(quat[2]),
        "qw": float(quat[3]),
        "corners": det.corners.tolist()
    })
    print(f"   Tag ID {det.tag_id}: t=[{pose_t[0]:.3f}, {pose_t[1]:.3f}, {pose_t[2]:.3f}]")

# Draw detections on image
img_with_tags = camera_image.copy()
for det in detections:
    corners = det.corners.astype(np.int32)
    cv2.polylines(img_with_tags, [corners], True, (0, 255, 0), 2)
    cv2.putText(img_with_tags, f"ID: {det.tag_id}", 
                tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 
                0.7, (0, 255, 0), 2)

cv2.imwrite(OUTPUT_IMAGE_WITH_TAGS, img_with_tags)
print(f"💾 Saved image with tags → {OUTPUT_IMAGE_WITH_TAGS}")

# ============================================================
# 📥 Extract Synchronized LiDAR Data (OPTIMIZED)
# ============================================================

print("⏳ Extracting synchronized LiDAR pointcloud...")
print("   (This may take 5-10 minutes for the 135GB MCAP bag...)")

reader2 = rosbag2_py.SequentialReader()
reader2.open(storage, converter)

PointCloud2Type = get_message(type_map[LIDAR_TOPIC])

min_time_diff = float('inf')
closest_lidar_points = None
lidar_timestamp = None

msg_count = 0
lidar_count = 0

while reader2.has_next():
    topic, data, stamp = reader2.read_next()
    msg_count += 1
    
    # Progress indicator every 10k messages
    if msg_count % 10000 == 0:
        print(f"   Processed {msg_count} messages, found {lidar_count} LiDAR scans...")
    
    if topic == LIDAR_TOPIC:
        lidar_count += 1
        time_diff = abs(stamp - target_timestamp)
        
        if time_diff < min_time_diff:
            min_time_diff = time_diff
            lidar_timestamp = stamp
            
            msg = deserialize_message(data, PointCloud2Type)
            
            # Parse PointCloud2
            point_step = msg.point_step
            num_points = msg.width * msg.height
            points = []
            
            for i in range(num_points):
                offset = i * point_step
                x = np.frombuffer(msg.data[offset:offset+4], dtype=np.float32)[0]
                y = np.frombuffer(msg.data[offset+4:offset+8], dtype=np.float32)[0]
                z = np.frombuffer(msg.data[offset+8:offset+12], dtype=np.float32)[0]
                points.append([x, y, z])
            
            closest_lidar_points = np.array(points)
            
            # Early exit if we found a very close match (< 50ms)
            if time_diff < 50_000_000:
                print(f"   Found excellent sync (Δ{time_diff/1e6:.2f}ms), stopping early")
                break

print(f"✅ Got {len(closest_lidar_points)} LiDAR points (time diff: {min_time_diff/1e9:.3f}s)")

# ============================================================
# 💾 Save All Data
# ============================================================

# Save frame data summary
frame_data = {
    "image_index": selected_index,
    "image_filename": image_filename,
    "image_timestamp_ns": target_timestamp,
    "lidar_timestamp_ns": int(lidar_timestamp),
    "time_sync_diff_ms": min_time_diff / 1e6,
    "camera_K": camera_K.tolist(),
    "camera_D": camera_D.tolist(),
    "num_detections": len(detections_list),
    "num_lidar_points": len(closest_lidar_points)
}

with open(OUTPUT_FRAME_DATA, "w") as f:
    json.dump(frame_data, f, indent=2)
print(f"💾 Saved frame data → {OUTPUT_FRAME_DATA}")

# Save detections
with open(OUTPUT_DETECTIONS, "w") as f:
    json.dump(detections_list, f, indent=2)
print(f"💾 Saved detections → {OUTPUT_DETECTIONS}")

# Save pointcloud
np.save(OUTPUT_POINTCLOUD, closest_lidar_points)
print(f"💾 Saved pointcloud → {OUTPUT_POINTCLOUD}")

print("\n🎯 Step 2 complete!")
print(f"\nExtracted data for image {selected_index}:")
print(f"   - {len(detections_list)} AprilTag detection(s)")
print(f"   - {len(closest_lidar_points)} LiDAR points")
print(f"   - Time sync difference: {min_time_diff/1e6:.2f} ms")
print(f"\nNext: Run step 3 to project and visualize in Rerun")
