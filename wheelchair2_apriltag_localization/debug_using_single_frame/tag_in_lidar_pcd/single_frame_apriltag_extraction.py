#!/usr/bin/env python3
"""
Step 1: Extract ALL images from MCAP rosbag with timestamps
Creates: images/ folder with numbered images and image_timestamps.json

Author: Akhila shaik
Date: 2025-12-16
"""

import rosbag2_py
import json
import os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import cv2
from cv_bridge import CvBridge

# ============================================================
# 🧩 CONFIGURATION
# ============================================================

BAG_PATH = "/media/akhila/Expansion/SARA/lab_markers/2025-10-17_01-31-14/rosbag/"
STORAGE_ID = "mcap"  # Changed from sqlite3
IMAGE_TOPIC = "/camera2/camera2/color/image_raw"  # Changed from camera1 to camera2

OUTPUT_DIR = "images"
OUTPUT_TIMESTAMPS = "image_timestamps.json"

# ============================================================
# 📥 Open Rosbag
# ============================================================

print(f"🔍 Opening bag: {BAG_PATH}")
reader = rosbag2_py.SequentialReader()
storage = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id=STORAGE_ID)
converter = rosbag2_py.ConverterOptions('', '')
reader.open(storage, converter)

topics = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topics}

print(f"📋 Available topics:")
for topic_name in type_map.keys():
    print(f"   - {topic_name}")

if IMAGE_TOPIC not in type_map:
    print(f"❌ Error: Topic {IMAGE_TOPIC} not found in bag!")
    exit(1)

ImageType = get_message(type_map[IMAGE_TOPIC])
bridge = CvBridge()

# Create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============================================================
# 📸 Extract All Images
# ============================================================

print(f"📸 Extracting all images from {IMAGE_TOPIC}...")
print(f"   Expected: ~27,395 images")

image_data = []
count = 0

while reader.has_next():
    topic, data, stamp = reader.read_next()
    
    if topic == IMAGE_TOPIC:
        msg = deserialize_message(data, ImageType)
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Save image with index
        image_filename = f"image_{count:06d}.jpg"
        image_path = os.path.join(OUTPUT_DIR, image_filename)
        cv2.imwrite(image_path, img)
        
        # Store timestamp info
        image_data.append({
            "index": count,
            "filename": image_filename,
            "timestamp_ns": int(stamp),
            "timestamp_sec": stamp / 1e9
        })
        
        count += 1
        
        if count % 500 == 0:
            print(f"   Extracted {count} images...")

print(f"✅ Extracted {count} total images")

# ============================================================
# 💾 Save Timestamp Mapping
# ============================================================

with open(OUTPUT_TIMESTAMPS, "w") as f:
    json.dump(image_data, f, indent=2)

print(f"💾 Saved timestamp info → {OUTPUT_TIMESTAMPS}")
print(f"💾 Saved images → {OUTPUT_DIR}/")

print(f"\n🎯 Step 1 complete!")
print(f"\nNext steps:")
print(f"1. Look through images in '{OUTPUT_DIR}/' folder")
print(f"2. Choose one image with good AprilTag view (note the index number)")
print(f"3. Run step 2 with that image index:")
print(f"   python step2_process_selected_image.py <index>")
print(f"\n💡 Tip: Use an image viewer or write a quick preview script to browse images")
