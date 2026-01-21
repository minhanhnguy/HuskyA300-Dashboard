
import sys
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_description(bag_path, output_file):
    print(f"Extracting description from: {bag_path}")
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    StringMsg = get_message("std_msgs/msg/String")
    
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if "robot_description" in topic:
            msg = deserialize_message(raw, StringMsg)
            print(f"Found description! Length: {len(msg.data)}")
            with open(output_file, 'w') as f:
                f.write(msg.data)
            print(f"Saved to {output_file}")
            return

if __name__ == "__main__":
    bag_path = "/home/aminh/HuskyA300-Dashboard/public/bag_files/nopreview_of_map.mcap"
    output_file = "/home/aminh/HuskyA300-Dashboard/robot_description.urdf"
    extract_description(bag_path, output_file)
