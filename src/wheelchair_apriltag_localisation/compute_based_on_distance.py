#!/usr/bin/env python3
"""
Robust tag averaging with DISTANCE THRESHOLD filtering
Only uses detections within 3 meters of the camera for better accuracy
"""
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import defaultdict

# ============================================================
# CONFIGURATION
# ============================================================
POSES_JSON = "/media/akhila/Expansion/SARA/github/pose_image_sync.json"
DETECTIONS_JSON = "/media/akhila/Expansion/SARA/github/detections.json"
OUTPUT_JSON = "/media/akhila/Expansion/SARA/github/distanc3_tags.json"
OUTPUT_PLY = "/media/akhila/Expansion/SARA/github/disatnace3_tags.ply"

# Calibration
T_LIDAR_CAMERA = [
    0.06223598068742114,
    0.012237596587866426,
    -0.05778298390566573,
    0.5035505589671723,
    0.4987164201169012,
    0.4961475523030442,
    0.5015539584265382
]


# Filtering parameters
MAX_DISTANCE = 3  # Only use detections within 35
MAX_TIME_DIFF = 0.05        # 50ms time sync tolerance
MIN_DETECTIONS = 5         # Minimum detections needed per tag
OUTLIER_THRESHOLD = 2.5     # Outlier removal (σ)
MAX_ITERATIONS = 5          # Outlier removal iterations

print("="*60)
print("DISTANCE THRESHOLD TAG FILTERING (3 METERS)")
print("="*60)
print(f"\n📏 Distance threshold: {MAX_DISTANCE} meters")
print(f"⏱️  Max time diff:     {MAX_TIME_DIFF*1000:.0f} ms")
print(f"📊 Min detections:     {MIN_DETECTIONS}")
print(f"🎯 Outlier threshold:  {OUTLIER_THRESHOLD} σ")

# ============================================================
# HELPER FUNCTIONS
# ============================================================
def tum_to_matrix(params):
    """Convert TUM format [x,y,z,qx,qy,qz,qw] to 4x4 matrix"""
    x, y, z, qx, qy, qz, qw = params
    rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [x, y, z]
    return T

# ============================================================
# LOAD DATA
# ============================================================
with open(POSES_JSON) as f:
    poses_sync = json.load(f)
with open(DETECTIONS_JSON) as f:
    detections = json.load(f)

pose_times = np.array([p["image_time"] for p in poses_sync])
T_lidar_camera = tum_to_matrix(T_LIDAR_CAMERA)

print(f"\n✅ Loaded {len(poses_sync):,} poses")
print(f"✅ Loaded {len(detections):,} detections")

# ============================================================
# COMPUTE TAG POSITIONS WITH DISTANCE FILTERING
# ============================================================
tag_data = defaultdict(list)
distance_filtered = 0
time_filtered = 0

print("\nFiltering detections by distance...")
for i, det in enumerate(detections):
    if (i + 1) % 5000 == 0:
        print(f"  {i+1}/{len(detections)}...")
    
    # Calculate distance to tag in camera frame
    cam_dist = np.sqrt(det["tx"]**2 + det["ty"]**2 + det["tz"]**2)
    
    # FILTER 1: Distance threshold
    if cam_dist > MAX_DISTANCE:
        distance_filtered += 1
        continue
    
    # FILTER 2: Time synchronization
    idx = np.argmin(np.abs(pose_times - det["timestamp"]))
    time_diff = abs(pose_times[idx] - det["timestamp"])
    
    if time_diff > MAX_TIME_DIFF:
        time_filtered += 1
        continue
    
    # Get robot pose
    p = poses_sync[idx]["pose"]
    T_map_lidar = tum_to_matrix([
        p["x"], p["y"], p["z"],
        p["qx"], p["qy"], p["qz"], p["qw"]
    ])
    
    # Get tag pose in camera frame
    T_camera_tag = tum_to_matrix([
        det["tx"], det["ty"], det["tz"],
        det["qx"], det["qy"], det["qz"], det["qw"]
    ])
    
    # Transform to map frame
    T_map_tag = T_map_lidar @ T_lidar_camera @ T_camera_tag
    pos = T_map_tag[:3, 3]
    
    # Store detection
    tag_data[det["id"]].append({
        "position": pos,
        "timestamp": det["timestamp"],
        "time_diff": time_diff,
        "distance": cam_dist
    })

print(f"  {len(detections)}/{len(detections)}... Done!")

kept_count = sum(len(v) for v in tag_data.values())
total_filtered = distance_filtered + time_filtered

print(f"\n📊 Filtering Statistics:")
print(f"  🚫 Distance filtered (>{MAX_DISTANCE}m): {distance_filtered:,}")
print(f"  🚫 Time filtered (>{MAX_TIME_DIFF*1000}ms): {time_filtered:,}")
print(f"  ✅ Kept for processing:                {kept_count:,}")
print(f"  📉 Rejection rate:                     {100*total_filtered/len(detections):.1f}%")

# ============================================================
# OUTLIER REMOVAL AND AVERAGING
# ============================================================
final_tags = []

print("\n" + "="*60)
print("OUTLIER REMOVAL PER TAG")
print("="*60)

for tid in sorted(tag_data.keys()):
    data = tag_data[tid]
    
    if len(data) < MIN_DETECTIONS:
        print(f"❌ Tag {tid:2d}: Only {len(data):4d} detections within {MAX_DISTANCE}m (skipped)")
        continue
    
    positions = np.array([d["position"] for d in data])
    
    # Multi-iteration outlier removal
    inliers = positions.copy()
    inlier_data = data.copy()
    
    for iteration in range(MAX_ITERATIONS):
        if len(inliers) < MIN_DETECTIONS:
            break
        
        mean = np.mean(inliers, axis=0)
        distances = np.linalg.norm(inliers - mean, axis=1)
        std = np.std(distances)
        
        # Early stop if already very precise
        if std < 0.02:
            break
        
        # Remove outliers
        mask = distances < (OUTLIER_THRESHOLD * std)
        new_inliers = inliers[mask]
        
        # Stop if no more outliers found
        if len(new_inliers) == len(inliers):
            break
        
        inliers = new_inliers
        inlier_data = [d for i, d in enumerate(inlier_data) if mask[i]]
    
    if len(inliers) < MIN_DETECTIONS:
        print(f"❌ Tag {tid:2d}: After filtering, only {len(inliers)} inliers (skipped)")
        continue
    
    # Compute final statistics
    final_mean = np.mean(inliers, axis=0)
    final_std = np.std(inliers, axis=0)
    mean_std = np.mean(final_std)
    
    # Get distance statistics
    distances = np.array([d["distance"] for d in inlier_data])
    avg_distance = np.mean(distances)
    
    outliers_removed = len(positions) - len(inliers)
    removal_pct = 100 * outliers_removed / len(positions)
    
    flag = "✅" if mean_std < 0.15 else "⚠️ " if mean_std < 0.3 else "❌"
    print(f"{flag} Tag {tid:2d}: {len(inliers):4d}/{len(positions):4d} inliers "
          f"({removal_pct:4.1f}% removed), std={mean_std:.3f}m, avg_dist={avg_distance:.2f}m")
    
    final_tags.append({
        "id": tid,
        "x": float(final_mean[0]),
        "y": float(final_mean[1]),
        "z": float(final_mean[2]),
        "std_x": float(final_std[0]),
        "std_y": float(final_std[1]),
        "std_z": float(final_std[2]),
        "mean_std": float(mean_std),
        "count": len(inliers),
        "outliers_removed": outliers_removed,
        "avg_distance": float(avg_distance),
        "max_distance_used": MAX_DISTANCE
    })

# ============================================================
# SAVE RESULTS
# ============================================================
with open(OUTPUT_JSON, "w") as f:
    json.dump(final_tags, f, indent=2)

with open(OUTPUT_PLY, "w") as f:
    f.write("ply\nformat ascii 1.0\n")
    f.write(f"element vertex {len(final_tags)}\n")
    f.write("property float x\nproperty float y\nproperty float z\n")
    f.write("end_header\n")
    for t in final_tags:
        f.write(f"{t['x']} {t['y']} {t['z']}\n")

print("\n" + "="*60)
print("FINAL RESULTS")
print("="*60)

good_tags = [t for t in final_tags if t['mean_std'] < 0.15]
ok_tags = [t for t in final_tags if 0.15 <= t['mean_std'] < 0.3]
bad_tags = [t for t in final_tags if t['mean_std'] >= 0.3]

print(f"\n✅ Excellent tags (<0.15m): {len(good_tags)}")
if good_tags:
    print(f"   IDs: {[t['id'] for t in good_tags]}")
    avg_dist = np.mean([t['avg_distance'] for t in good_tags])
    print(f"   Avg detection distance: {avg_dist:.2f}m")

print(f"\n⚠️  Good tags (0.15-0.3m): {len(ok_tags)}")
if ok_tags:
    print(f"   IDs: {[t['id'] for t in ok_tags]}")
    avg_dist = np.mean([t['avg_distance'] for t in ok_tags])
    print(f"   Avg detection distance: {avg_dist:.2f}m")

print(f"\n❌ Poor tags (>0.3m): {len(bad_tags)}")
if bad_tags:
    print(f"   IDs: {[t['id'] for t in bad_tags]}")

print(f"\n💾 Saved {len(final_tags)} tags → {OUTPUT_JSON}")
print(f"💾 Saved PLY → {OUTPUT_PLY}")

print("\n📊 Overall Statistics:")
print(f"  Distance threshold used: {MAX_DISTANCE}m")
print(f"  Total tags processed: {len(tag_data)}")
print(f"  Tags with sufficient data: {len(final_tags)}")
print(f"  Success rate: {100*len(final_tags)/len(tag_data):.1f}%")

print("\n🎯 This method only uses close-range detections for maximum accuracy!")
print("="*60)
