
import sys
import os
import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message

def patch_bag(input_bag, output_bag, urdf_file):
    print(f"Patching bag: {input_bag} -> {output_bag}")
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    # Setup reader
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Setup writer
    writer = rosbag2_py.SequentialWriter()
    out_storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id="mcap")
    writer.open(out_storage_options, converter_options)
    
    # Create topics
    topics = reader.get_all_topics_and_types()
    topic_types = {}
    
    # Check if robot_description topic exists in input
    has_desc_topic = False
    desc_topic_name = "/a300_0000/robot_description"
    
    for topic in topics:
        writer.create_topic(topic)
        topic_types[topic.name] = topic.type
        if topic.name == desc_topic_name:
            has_desc_topic = True
            
    if not has_desc_topic:
        print(f"Creating topic {desc_topic_name}")
        desc_meta = rosbag2_py.TopicMetadata(
            name=desc_topic_name,
            type="std_msgs/msg/String",
            serialization_format="cdr"
        )
        writer.create_topic(desc_meta)

    # Get start time
    metadata = reader.get_metadata()
    start_time_ns = metadata.starting_time.nanoseconds
    
    # Create and write robot description message
    print("Inserting robot_description message at start...")
    StringMsg = get_message("std_msgs/msg/String")
    msg = StringMsg()
    msg.data = urdf_content
    
    # Write it slightly before the first message or at start time
    writer.write(desc_topic_name, serialize_message(msg), start_time_ns)
    
    # Copy all messages
    count = 0
    while reader.has_next():
        topic, raw, t = reader.read_next()
        writer.write(topic, raw, t)
        count += 1
        if count % 10000 == 0:
            print(f"Copied {count} messages...")
            
    print(f"Finished! Copied {count} messages.")

if __name__ == "__main__":
    input_bag = "/home/aminh/HuskyA300-Dashboard/public/bag_files/preview_of_map.mcap"
    output_bag = "/home/aminh/HuskyA300-Dashboard/public/bag_files/preview_of_map_fixed.mcap"
    urdf_file = "/home/aminh/HuskyA300-Dashboard/robot_description.urdf"
    
    if os.path.exists(output_bag):
        import shutil
        shutil.rmtree(output_bag)
        
    patch_bag(input_bag, output_bag, urdf_file)
