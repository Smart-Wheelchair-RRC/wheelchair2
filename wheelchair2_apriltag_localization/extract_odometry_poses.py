import rosbag2_py, json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/media/akhila/Expansion/SARA/lab_markers/2025-10-17_01-31-14/rosbag"
odom_topic = "/Odometry"

print(f"🔍 Reading odometry from: {odom_topic}")

reader = rosbag2_py.SequentialReader()
storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
converter = rosbag2_py.ConverterOptions('', '')
reader.open(storage, converter)

topics = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topics}
OdomType = get_message(type_map[odom_topic])

poses = []
count = 0
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == odom_topic:
        msg = deserialize_message(data, OdomType)
        poses.append({
            "timestamp": stamp,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w
        })
        count += 1
        if count % 500 == 0:
            print(f"Processed {count} messages...")

with open("poses.json", "w") as f:
    json.dump(poses, f, indent=2)
print(f"\n✅ Saved {len(poses)} poses → poses.json")
