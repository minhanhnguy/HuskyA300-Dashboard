
import sys
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def inspect_bag(bag_path):
    print(f"Inspecting bag: {bag_path}")
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    # Get metadata
    metadata = reader.get_metadata()
    print(f"\nDuration: {metadata.duration}")
    print(f"Start time: {metadata.starting_time}")
    print(f"Message count: {metadata.message_count}")
    
    print("\nTopics:")
    topics_info = reader.get_all_topics_and_types()
    topic_map = {}
    for info in topics_info:
        print(f"  {info.name} ({info.type})")
        print(f"    QoS: {info.offered_qos_profiles}")
        topic_map[info.name] = info.type

    # Check for specific topics
    print("\nKey Topics Check:")
    map_topics = [t for t in topic_map.keys() if "map" in t]
    print(f"  Map topics: {map_topics}")
    
    tf_topics = [t for t in topic_map.keys() if "tf" in t]
    print(f"  TF topics: {tf_topics}")
    
    desc_topics = [t for t in topic_map.keys() if "description" in t]
    print(f"  Description topics: {desc_topics}")

    return

    # Analyze TFs
    print("\nAnalyzing TF Tree (first 1000 messages)...")
    
    # Re-open to read messages
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    tf_frames = set()
    tf_tree = {} # parent -> children
    
    TFMessage = get_message("tf2_msgs/msg/TFMessage")
    
    count = 0
    while reader.has_next() and count < 5000:
        topic, raw, t = reader.read_next()
        count += 1
        
        if "tf" in topic:
            try:
                msg = deserialize_message(raw, TFMessage)
                for ts in msg.transforms:
                    parent = ts.header.frame_id
                    child = ts.child_frame_id
                    tf_frames.add(parent)
                    tf_frames.add(child)
                    
                    if parent not in tf_tree:
                        tf_tree[parent] = set()
                    tf_tree[parent].add(child)
            except Exception as e:
                pass

    print(f"  Frames found: {tf_frames}")
    print("  Tree structure:")
    for parent, children in tf_tree.items():
        print(f"    {parent} -> {list(children)}")

if __name__ == "__main__":
    bag_path = "/home/aminh/HuskyA300-Dashboard/public/bag_files/preview_of_map.mcap"
    inspect_bag(bag_path)
