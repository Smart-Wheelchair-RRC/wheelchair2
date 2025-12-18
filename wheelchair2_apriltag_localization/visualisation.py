import rerun as rr
import numpy as np
import json
import struct
import os

# =========================================================
# CONFIGURATION
# =========================================================
MAP_FILE = "/media/akhila/Expansion/SARA/github/RRC_april.pcd"  # binary or ascii
JSON_FILE = "/media/akhila/Expansion/SARA/github/distance3_tags.json"

# =========================================================
# POINT CLOUD LOADER (PCD only)
# =========================================================
def load_pcd(filename):
    """
    Load a .pcd file (binary or ascii) manually.
    Supports typical XYZ / XYZRGB formats.
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"PCD file not found: {filename}")

    with open(filename, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header.append(line)
            if line.startswith('DATA'):
                data_type = line.split()[1].lower()
                break

        # Parse header
        fields = []
        size = []
        count = []
        points_count = 0
        for line in header:
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                size = list(map(int, line.split()[1:]))
            elif line.startswith('COUNT'):
                count = list(map(int, line.split()[1:]))
            elif line.startswith('POINTS'):
                points_count = int(line.split()[1])

        # ASCII data
        if data_type == 'ascii':
            data = []
            for line in f:
                parts = line.decode('ascii', errors='ignore').strip().split()
                if len(parts) >= 3:
                    data.append([float(parts[0]), float(parts[1]), float(parts[2])])
            return np.array(data, dtype=np.float32)

        # BINARY data
        elif data_type == 'binary':
            print("Detected binary PCD — decoding...")
            binary = f.read()
            total_bytes = len(binary)
            field_bytes = sum([s * c for s, c in zip(size, count)])
            expected_bytes = field_bytes * points_count

            if total_bytes < expected_bytes:
                print(f"⚠️ Warning: binary file smaller than expected ({total_bytes} < {expected_bytes})")

            # We only care about x,y,z (first 3 floats)
            step = sum(size)
            pts = []
            offset = 0
            for _ in range(points_count):
                x, y, z = struct.unpack('fff', binary[offset:offset+12])
                pts.append((x, y, z))
                offset += step
            return np.array(pts, dtype=np.float32)

        else:
            raise ValueError(f"Unsupported DATA format: {data_type}")

# =========================================================
# LOAD TAGS
# =========================================================
def load_tags(json_path):
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"Tag JSON not found: {json_path}")
    with open(json_path, "r") as f:
        #tags = json.load(f)
        tags = json.load(open(JSON_FILE, "r", encoding="utf-8-sig"))

    print(f"✅ Loaded {len(tags)} tags")
    return tags

# =========================================================
# MAIN VISUALIZATION
# =========================================================
print("🚀 Initializing Rerun...")
rr.init("aruco_map_visualization", spawn=True)

# 1️⃣ Load Map
try:
    print(f"📂 Loading map: {MAP_FILE}")
    map_points = load_pcd(MAP_FILE)
    print(f"✅ Loaded {len(map_points):,} points")
    map_colors = np.ones((len(map_points), 3), dtype=np.uint8) * 120  # gray
except Exception as e:
    print(f"❌ Failed to load map: {e}")
    print("Using dummy points for visualization.")
    map_points = np.random.rand(1000, 3) * 10
    map_colors = np.ones((len(map_points), 3), dtype=np.uint8) * 120

# Log map
rr.log("world/map", rr.Points3D(map_points, colors=map_colors, radii=0.01))

# 2️⃣ Load Tags
tags = load_tags(JSON_FILE)

# Bright colors
BRIGHT_COLORS = np.array([
    [255, 0, 0], [0, 255, 0], [0, 0, 255],
    [255, 255, 0], [255, 0, 255], [0, 255, 255],
    [255, 128, 0], [128, 0, 255], [255, 0, 128],
    [0, 255, 128], [128, 255, 0], [0, 128, 255],
    [255, 128, 128], [128, 255, 128], [128, 128, 255]
], dtype=np.uint8)

# 3️⃣ Visualize Tags
print("\n📍 Plotting ArUco Tags...")
for i, tag in enumerate(tags):
    tag_id = tag.get("id", i)
    pos = np.array([tag.get("x", 0.0), tag.get("y", 0.0), tag.get("z", 0.0)], dtype=np.float32)
    count = tag.get("count", 0)
    color = BRIGHT_COLORS[i % len(BRIGHT_COLORS)]

    print(f"  ↳ Tag {tag_id:>3}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) detections={count}")

    rr.log(f"world/tags/tag_{tag_id}/position",
           rr.Points3D([pos], colors=[color], radii=0.08))
    rr.log(f"world/tags/tag_{tag_id}/label",
           rr.TextDocument(f"Tag {tag_id}\nDetections: {count}"))
    rr.log(f"world/tags/tag_{tag_id}/bbox",
           rr.Boxes3D(half_sizes=[[0.075, 0.075, 0.005]],
                      centers=[pos], colors=[color]))
    rr.log(f"world/tags/tag_{tag_id}/frame",
           rr.Transform3D(translation=pos))

print("\n" + "="*60)
print("✅ Visualization Ready in Rerun")
print("="*60)
print("\n🧭 Controls:")
print("  • Left Mouse  → Rotate")
print("  • Right Mouse → Pan")
print("  • Scroll      → Zoom")
print("  • Press F     → Focus on map or tag\n")
print(f"  • Map: {len(map_points):,} points")
print(f"  • Tags: {len(tags)} total\n")
