# ================== extract_images_sqlite3.py ==================
import rosbag2_py, cv2, os, json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# -------------------- CONFIG --------------------------
bag_path = "/home/akhila/apriltag_synced_bag_fresh"
image_topic = "/camera2/camera2/color/image_raw"   # check with ros2 bag info
output_dir = "images"
os.makedirs(output_dir, exist_ok=True)
out_json = "image_timestamps.json"

# -------------------- SETUP READER --------------------
reader = rosbag2_py.SequentialReader()
storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")  # ✅ FIXED
converter = rosbag2_py.ConverterOptions('', '')
reader.open(storage, converter)

topics = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topics}
if image_topic not in type_map:
    print("❌ Topic not found. Available topics:")
    for t in type_map:
        print("  ", t)
    raise SystemExit

ImgType = get_message(type_map[image_topic])
bridge = CvBridge()

# -------------------- EXTRACT -------------------------
frames = []
count = 0
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic != image_topic:
        continue
    msg = deserialize_message(data, ImgType)
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    fname = f"frame_{count:05d}.jpg"
    cv2.imwrite(os.path.join(output_dir, fname), img)

    frames.append({"frame": fname, "timestamp": ts})
    count += 1
    if count % 500 == 0:
        print(f"Saved {count} frames...")

# -------------------- SAVE ----------------------------
with open(out_json, "w") as f:
    json.dump(frames, f, indent=2)

print(f"✅ Extracted {len(frames)} frames → {output_dir}/")
print(f"🕒 Image timestamps saved → {out_json}")
