#!/usr/bin/env python3
"""
Extract AprilTag 3D detections (camera→tag transforms) from /tf
and save them as detections.json.

Author: Akhila shaik
Date: 2025-11-12
"""

import rosbag2_py
import json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ============================================================
# 🧩 CONFIGURATION
# ============================================================

BAG_PATH = "/home/akhila/apriltag_synced_bag_fresh/"  # <-- your bag path
TF_TOPIC = "/tf"
OUTPUT_JSON = "detections.json"

# ============================================================
# 🧠 OPEN BAG
# ============================================================

print(f"🔍 Reading TF topic from: {BAG_PATH}")
reader = rosbag2_py.SequentialReader()
storage = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="sqlite3")
converter = rosbag2_py.ConverterOptions("", "")
reader.open(storage, converter)

topics = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topics}
if TF_TOPIC not in type_map:
    raise RuntimeError(f"❌ Topic {TF_TOPIC} not found in bag!")

TFType = get_message(type_map[TF_TOPIC])

# ============================================================
# 🧩 READ TF MESSAGES
# ============================================================

detections = []
count = 0

print("⏳ Extracting AprilTag transforms from /tf ...")

while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic != TF_TOPIC:
        continue

    msg = deserialize_message(data, TFType)

    for tf in msg.transforms:
        parent = tf.header.frame_id.lower()
        child = tf.child_frame_id.lower()

        # Only take transforms published by the camera frame
        # and whose child frame corresponds to an AprilTag
        if not parent.startswith("camera2") or "tag" not in child:
            continue

        # Example child_frame_id: "tag36h11:8"
        try:
            tag_id = int(child.split(":")[-1])
        except Exception:
            continue  # skip malformed names

        ts = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        t = tf.transform.translation
        r = tf.transform.rotation

        detections.append({
            "timestamp": ts,
            "id": tag_id,
            "tx": t.x,
            "ty": t.y,
            "tz": t.z,
            "qx": r.x,
            "qy": r.y,
            "qz": r.z,
            "qw": r.w
        })

        count += 1
        if count % 500 == 0:
            print(f"   → Processed {count} detections...")

print(f"✅ Finished extracting {count} tag detections.")

# ============================================================
# 💾 SAVE JSON
# ============================================================

with open(OUTPUT_JSON, "w") as f:
    json.dump(detections, f, indent=2)

print(f"💾 Saved detections → {OUTPUT_JSON}")
