import json, bisect, statistics

# ----------------------------
# Load input JSON files
# ----------------------------
with open("poses.json") as f:
    poses = json.load(f)
with open("image_timestamps.json") as f:
    images = json.load(f)

print(f"Loaded {len(poses)} poses and {len(images)} image timestamps")

# ----------------------------
# Normalize timestamps
# ----------------------------
# poses → convert nanoseconds to seconds
for p in poses:
    if p["timestamp"] > 1e12:  # if still in ns
        p["timestamp"] = p["timestamp"] * 1e-9

pose_times = [p["timestamp"] for p in poses]

# ----------------------------
# Synchronize each image with nearest pose
# ----------------------------
synced = []
time_diffs = []

for img in images:
    t_img = img["timestamp"]
    idx = bisect.bisect_left(pose_times, t_img)

    # Find nearest odometry
    if idx == 0:
        nearest = poses[0]
    elif idx >= len(pose_times):
        nearest = poses[-1]
    else:
        before, after = poses[idx-1], poses[idx]
        nearest = before if abs(before["timestamp"] - t_img) < abs(after["timestamp"] - t_img) else after

    dt = abs(nearest["timestamp"] - t_img)
    time_diffs.append(dt)

    synced.append({
        "image": img["frame"],
        "image_time": t_img,
        "pose_time": nearest["timestamp"],
        "dt_sec": dt,
        "pose": {
            "x": nearest["x"],
            "y": nearest["y"],
            "z": nearest["z"],
            "qx": nearest["qx"],
            "qy": nearest["qy"],
            "qz": nearest["qz"],
            "qw": nearest["qw"]
        }
    })

# ----------------------------
# Save synchronized output
# ----------------------------
with open("pose_image_sync.json", "w") as f:
    json.dump(synced, f, indent=2)

# ----------------------------
# Print statistics
# ----------------------------
avg_dt = statistics.mean(time_diffs)
max_dt = max(time_diffs)
print(f"\n✅ Synced {len(synced)} image–pose pairs → pose_image_sync.json")
print(f"Average time difference: {avg_dt*1000:.2f} ms")
print(f"Max time difference: {max_dt*1000:.2f} ms")
