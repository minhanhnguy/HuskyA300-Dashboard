import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def inspect_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    print(f"Inspecting bag: {bag_path}")
    print("Looking for 'robot_description' in topics...")
    
    found = False
    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if "robot_description" in topic:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            print(f"Found '{topic}' at t={t_ns / 1e9:.3f}s")
            print(f"  Type: {type_map[topic]}")
            if hasattr(msg, 'data'):
                print(f"  Content length: {len(msg.data)} chars")
                print(f"  Preview: {msg.data[:100]}...")
            found = True
            # break # Keep searching to see if it's published multiple times

    if not found:
        print("No 'robot_description' topic found in bag.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 inspect_bag.py <path_to_bag>")
        sys.exit(1)
    inspect_bag(sys.argv[1])
