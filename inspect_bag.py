import sys
import os
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
import numpy as np

def flatten_message(msg, parent_key='', sep='.'):
    """
    Recursively flatten a ROS message into a dictionary.
    """
    items = []
    # Convert message to ordered dict first to handle basic types and nested structures
    msg_dict = message_to_ordereddict(msg)
    
    def _flatten(d, new_key):
        if isinstance(d, dict):
            for k, v in d.items():
                _flatten(v, new_key + sep + k if new_key else k)
        elif isinstance(d, list):
            for i, v in enumerate(d):
                _flatten(v, new_key + sep + str(i) if new_key else str(i))
        elif isinstance(d, np.ndarray):
             # Handle numpy arrays (often used for large data like images/lidar)
             # For CSV, we might want to truncate or summarize, but for now let's listify
             # If it's too large, this might be slow/huge.
             # For now, let's just convert to list and recurse or stringify?
             # Let's treat it as a list.
             _flatten(d.tolist(), new_key)
        else:
            items.append((new_key, d))

    _flatten(msg_dict, parent_key)
    return dict(items)

def inspect_bag(bag_path):
    bag_name = os.path.basename(bag_path)
    output_dir = f"{bag_name}_csv"
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")
    else:
        print(f"Output directory already exists: {output_dir}")

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    print(f"Exporting bag: {bag_path} to {output_dir}/")

    # Dictionary to hold CSV writers and file handles: {topic_name: {'file': f, 'writer': w, 'headers': []}}
    topic_writers = {}
    
    count = 0
    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        
        try:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
        except Exception as e:
            print(f"Failed to deserialize message on topic {topic}: {e}")
            continue

        # Flatten message
        flat_msg = flatten_message(msg)
        # Add timestamp
        flat_msg['timestamp'] = t_ns

        # Sanitize topic name for filename
        safe_topic_name = topic.replace('/', '_').strip('_')
        if not safe_topic_name:
            safe_topic_name = "root"
            
        if topic not in topic_writers:
            # First time seeing this topic, initialize CSV
            filename = os.path.join(output_dir, f"{safe_topic_name}.csv")
            
            # Collect all keys as headers. 
            # Note: This assumes the first message has all fields. 
            # If fields are optional/dynamic, this might miss some later ones or fail.
            # For standard ROS msgs, structure is usually fixed.
            headers = ['timestamp'] + [k for k in flat_msg.keys() if k != 'timestamp']
            
            f = open(filename, 'w', newline='')
            w = csv.DictWriter(f, fieldnames=headers)
            w.writeheader()
            
            topic_writers[topic] = {'file': f, 'writer': w, 'headers': headers}
            print(f"Started writing {topic} to {filename}")

        # Write row
        # Filter keys to match headers (in case of variable fields, though unlikely for standard msgs)
        # or fill missing with None
        row = {k: flat_msg.get(k) for k in topic_writers[topic]['headers']}
        topic_writers[topic]['writer'].writerow(row)
        
        count += 1
        if count % 1000 == 0:
            print(f"Processed {count} messages...", end='\r')

    print(f"\nFinished processing {count} messages.")

    # Close all files
    for t in topic_writers:
        topic_writers[t]['file'].close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 inspect_bag.py <path_to_bag>")
        sys.exit(1)
    inspect_bag(sys.argv[1])
