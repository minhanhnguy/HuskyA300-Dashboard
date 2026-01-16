import argparse
import csv
import os
import sys
from typing import List, Dict, Any
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

def flatten_message(msg_dict: Dict[str, Any], parent_key: str = '', sep: str = '_') -> Dict[str, Any]:
    """
    Recursively flattens a nested dictionary.
    Arrays > 20 elements are summarized.
    """
    items = []
    for k, v in msg_dict.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        
        if isinstance(v, dict):
            items.extend(flatten_message(v, new_key, sep=sep).items())
        elif isinstance(v, (list, tuple, np.ndarray)):
            if len(v) > 20:
                items.append((new_key, f"<array len={len(v)}>"))
            else:
                # For small arrays, we can either join them or expand them.
                # Expanding might create too many columns if size varies.
                # Joining is safer for CSV.
                items.append((new_key, str(v)))
        else:
            items.append((new_key, v))
    return dict(items)

def extract_bag_to_csvs(bag_path: str, output_dir: str):
    """
    Extracts ALL topics from a bag file into separate CSVs in the output directory.
    """
    os.makedirs(output_dir, exist_ok=True)
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag {bag_path}: {e}")
        return

    # Get all topics and types
    topic_types = {}
    for topic_metadata in reader.get_all_topics_and_types():
        topic_types[topic_metadata.name] = topic_metadata.type

    print(f"Reading bag: {bag_path}")
    print(f"Writing to: {output_dir}")
    
    # Writers cache: topic_name -> (file_handle, csv_writer, fieldnames_set)
    writers = {}
    
    count = 0
    
    while reader.has_next():
        topic, raw_data, t_ns = reader.read_next()
        msg_type_name = topic_types.get(topic)
        t_sec = t_ns / 1e9
        
        try:
            msg_class = get_message(msg_type_name)
            msg = deserialize_message(raw_data, msg_class)
            msg_dict = message_to_ordereddict(msg)
            
            # Flatten message
            flat_msg = flatten_message(msg_dict)
            flat_msg["Timestamp"] = t_sec
            
            # Sanitize topic name for filename
            safe_topic_name = topic.replace("/", "_").strip("_")
            if not safe_topic_name:
                safe_topic_name = "root"
                
            if topic not in writers:
                # First time seeing this topic, create writer
                csv_path = os.path.join(output_dir, f"{safe_topic_name}.csv")
                f = open(csv_path, 'w', newline='')
                
                # Determine fieldnames from the first message
                # Note: If subsequent messages have different fields (optional fields), this might break or miss data.
                # But for ROS msgs, structure is usually static.
                fieldnames = ["Timestamp"] + [k for k in flat_msg.keys() if k != "Timestamp"]
                
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writers[topic] = (f, writer, set(fieldnames))
            
            f, writer, fieldnames = writers[topic]
            
            # Filter row to only include known fields to avoid ValueError
            row_to_write = {k: v for k, v in flat_msg.items() if k in fieldnames}
            writer.writerow(row_to_write)
                
        except Exception as e:
            # print(f"Error processing {topic}: {e}")
            pass
            
        count += 1
        if count % 10000 == 0:
            print(f"Processed {count} messages...")

    # Close all files
    for f, _, _ in writers.values():
        f.close()
        
    print(f"Finished. Processed {count} messages. Created {len(writers)} CSV files.")

def main():
    parser = argparse.ArgumentParser(description="Extract ALL bag data to separate CSVs.")
    parser.add_argument("original_bag", help="Path to the original bag file")
    parser.add_argument("spoofed_bag", help="Path to the spoofed bag file")
    parser.add_argument("--output_dir", default="bag_data_export", help="Output directory")
    
    args = parser.parse_args()
    
    # Original
    if os.path.exists(args.original_bag):
        extract_bag_to_csvs(args.original_bag, os.path.join(args.output_dir, "original"))
    else:
        print(f"Original bag not found: {args.original_bag}")

    # Spoofed
    if os.path.exists(args.spoofed_bag):
        extract_bag_to_csvs(args.spoofed_bag, os.path.join(args.output_dir, "spoofed"))
    else:
        print(f"Spoofed bag not found: {args.spoofed_bag}")

if __name__ == "__main__":
    main()
