
import sys
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def inspect_tf_static(bag_path):
    print(f"Inspecting tf_static in: {bag_path}")
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    TFMessage = get_message("tf2_msgs/msg/TFMessage")
    
    transforms = set()
    
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if "tf_static" in topic:
            msg = deserialize_message(raw, TFMessage)
            for ts in msg.transforms:
                key = f"{ts.header.frame_id} -> {ts.child_frame_id}"
                transforms.add(key)
                
    print(f"Found {len(transforms)} static transforms:")
    for t in sorted(transforms):
        print(f"  {t}")
    return transforms

if __name__ == "__main__":
    bag1 = "/home/aminh/HuskyA300-Dashboard/public/bag_files/preview_of_map.mcap"
    bag2 = "/home/aminh/HuskyA300-Dashboard/public/bag_files/nopreview_of_map.mcap"
    
    t1 = inspect_tf_static(bag1)
    print("-" * 50)
    t2 = inspect_tf_static(bag2)
    
    print("-" * 50)
    missing = t2 - t1
    if missing:
        print(f"Missing transforms in preview_of_map: {len(missing)}")
        for t in missing:
            print(f"  {t}")
    else:
        print("No missing static transforms found.")
