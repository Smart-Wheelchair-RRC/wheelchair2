#!/usr/bin/env python3
"""
Step 3: Visualize AprilTag projection to LiDAR frame in Rerun
Loads data from Step 2 and visualizes the projection

Author: Akhila Shaik
Date: 2025-12-16
"""

import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import rerun as rr
import cv2

# ============================================================
# 🧩 CONFIGURATION
# ============================================================

# Input files from Step 2
INPUT_FRAME_DATA = "selected_frame_data.json"
INPUT_IMAGE = "selected_image_with_tags.jpg"
INPUT_POINTCLOUD = "frame_pointcloud.npy"
INPUT_DETECTIONS = "detections.json"

# Extrinsic calibration: T_lidar→camera [x, y, z, qx, qy, qz, qw]
# CHANGE THIS to your camera's extrinsics!
T_LIDAR_CAMERA = [
    0.06223598068742114,
    0.012237596587866426,
    -0.05778298390566573,
    0.5035505589671723,
    0.4987164201169012,
    0.4961475523030442,
    0.5015539584265382
]

# ============================================================
# 🧮 Helper Functions
# ============================================================

def tum_to_matrix(extrinsics):
    """Convert [x, y, z, qx, qy, qz, qw] → 4x4 transform matrix"""
    x, y, z, qx, qy, qz, qw = extrinsics
    rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [x, y, z]
    return T

# ============================================================
# 📥 Load Data from Step 2
# ============================================================

print("📥 Loading data from Step 2...")

try:
    # Load frame metadata
    with open(INPUT_FRAME_DATA, "r") as f:
        frame_data = json.load(f)
    print(f"✅ Frame data: image {frame_data['image_index']}")
    
    # Load LiDAR pointcloud
    lidar_points = np.load(INPUT_POINTCLOUD)
    print(f"✅ LiDAR: {len(lidar_points):,} points")
    
    # Load AprilTag detections (in camera frame)
    with open(INPUT_DETECTIONS, "r") as f:
        detections = json.load(f)
    print(f"✅ Detections: {len(detections)} AprilTag(s)")
    
    # Load camera image
    camera_image = cv2.imread(INPUT_IMAGE)
    print(f"✅ Camera image loaded")
    
except FileNotFoundError as e:
    print(f"\n❌ Error: Could not find input files!")
    print(f"   Make sure you ran Step 2 first: python step2_detect_and_save.py <image_index>")
    print(f"   Missing file: {e.filename}")
    exit(1)

# ============================================================
# 🔄 Project AprilTags: Camera → LiDAR
# ============================================================

print("\n🔄 Projecting AprilTags to LiDAR frame...")
print(f"   Transform chain: AprilTag → Camera → LiDAR")

T_lidar_camera = tum_to_matrix(T_LIDAR_CAMERA)

tag_projections = []

for det in detections:
    # Build T_camera_tag from detection
    T_camera_tag = np.eye(4)
    quat = [det["qx"], det["qy"], det["qz"], det["qw"]]
    T_camera_tag[:3, :3] = R.from_quat(quat).as_matrix()
    T_camera_tag[:3, 3] = [det["tx"], det["ty"], det["tz"]]
    
    # Apply transform: T_lidar_tag = T_lidar_camera @ T_camera_tag
    T_lidar_tag = T_lidar_camera @ T_camera_tag
    
    tag_pos_lidar = T_lidar_tag[:3, 3]
    tag_rot_lidar = T_lidar_tag[:3, :3]
    
    tag_projections.append({
        'id': det['id'],
        'position_camera': [det["tx"], det["ty"], det["tz"]],
        'position_lidar': tag_pos_lidar,
        'rotation_lidar': tag_rot_lidar
    })
    
    print(f"\n   Tag {det['id']}:")
    print(f"      Camera: [{det['tx']:6.3f}, {det['ty']:6.3f}, {det['tz']:6.3f}] m")
    print(f"      LiDAR:  [{tag_pos_lidar[0]:6.3f}, {tag_pos_lidar[1]:6.3f}, {tag_pos_lidar[2]:6.3f}] m")

# ============================================================
# 🎨 Visualize in Rerun
# ============================================================

print(f"\n🎨 Launching Rerun visualization...")
rr.init("apriltag_lidar_projection", spawn=True)

# Log LiDAR pointcloud (gray)
rr.log("world/lidar/pointcloud", 
       rr.Points3D(lidar_points, radii=0.01, colors=[180, 180, 180]))

# Log LiDAR origin (cyan)
rr.log("world/lidar/origin", 
       rr.Points3D([[0, 0, 0]], radii=0.06, colors=[0, 255, 255]))

# Log camera position in LiDAR frame (yellow)
camera_origin = T_lidar_camera[:3, 3]
rr.log("world/camera/origin", 
       rr.Points3D([camera_origin], radii=0.06, colors=[255, 255, 0]))

# Log camera coordinate axes
cam_axis_len = 0.15
for axis_idx, (axis_name, color) in enumerate([('x', [255, 100, 100]), 
                                                 ('y', [100, 255, 100]), 
                                                 ('z', [100, 100, 255])]):
    axis_vec = np.zeros(3)
    axis_vec[axis_idx] = cam_axis_len
    axis_end = camera_origin + T_lidar_camera[:3, :3] @ axis_vec
    rr.log(f"world/camera/{axis_name}_axis",
           rr.LineStrips3D([[camera_origin, axis_end]], colors=[color]))

# Log camera image with detections
rr.log("camera/image", 
       rr.Image(cv2.cvtColor(camera_image, cv2.COLOR_BGR2RGB)))

# Log AprilTag projections in LiDAR frame
for tag in tag_projections:
    tag_id = tag['id']
    pos = tag['position_lidar']
    rot = tag['rotation_lidar']
    
    # Tag center (large red sphere)
    rr.log(f"world/apriltags/tag_{tag_id}/center",
           rr.Points3D([pos], radii=0.06, colors=[255, 0, 0]))
    
    # Tag coordinate axes (RGB = XYZ)
    axis_length = 0.12
    for axis_idx, (axis_name, color) in enumerate([('x', [255, 0, 0]), 
                                                     ('y', [0, 255, 0]), 
                                                     ('z', [0, 0, 255])]):
        axis_vec = np.zeros(3)
        axis_vec[axis_idx] = axis_length
        axis_end = pos + rot @ axis_vec
        rr.log(f"world/apriltags/tag_{tag_id}/{axis_name}_axis",
               rr.LineStrips3D([[pos, axis_end]], colors=[color], radii=0.003))
    
    # Text label
    rr.log(f"world/apriltags/tag_{tag_id}/label",
           rr.TextDocument(f"Tag {tag_id}\nLiDAR: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"))

# ============================================================
# 📊 Summary
# ============================================================

print("\n" + "="*60)
print("🎯 Visualization Complete!")
print("="*60)

print(f"\n📊 Summary:")
print(f"   • Image: {frame_data['image_filename']}")
print(f"   • Detected tags: {len(detections)}")
print(f"   • LiDAR points: {len(lidar_points):,}")
print(f"   • Sync quality: {frame_data['time_sync_diff_ms']:.2f} ms")

print(f"\n🔍 Rerun Viewer Guide:")
print(f"   • Gray points = LiDAR pointcloud")
print(f"   • Red spheres = AprilTag centers in LiDAR frame")
print(f"   • RGB axes on tags = Tag orientation (R=X, G=Y, B=Z)")
print(f"   • Cyan sphere = LiDAR origin (0,0,0)")
print(f"   • Yellow sphere = Camera position")
print(f"   • Light RGB axes = Camera orientation")

print(f"\n✅ Verification Checklist:")
print(f"   1. Do AprilTag positions align with pointcloud features?")
print(f"   2. Are tag orientations sensible (facing camera)?")
print(f"   3. Does camera position make sense?")
print(f"   4. Is the distance scale correct?")

print(f"\n💡 If projection looks wrong:")
print(f"   • Check T_LIDAR_CAMERA extrinsics in this script")
print(f"   • Verify TAG_SIZE was correct in Step 2 (currently 0.18m)")
print(f"   • Try a different image with clearer AprilTag view")

print("="*60)
