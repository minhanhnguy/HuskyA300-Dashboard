
import sys
import os
import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message

def patch_tf_static(input_bag, output_bag, source_bag):
    print(f"Patching bag: {input_bag} -> {output_bag}")
    print(f"Using static transforms from: {source_bag}")
    
    # 1. Extract static transforms from source bag
    source_storage = rosbag2_py.StorageOptions(uri=source_bag, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    source_reader = rosbag2_py.SequentialReader()
    source_reader.open(source_storage, converter_options)
    
    TFMessage = get_message("tf2_msgs/msg/TFMessage")
    static_transforms = []
    
    while source_reader.has_next():
        topic, raw, t = source_reader.read_next()
        if "tf_static" in topic:
            msg = deserialize_message(raw, TFMessage)
            if msg.transforms:
                static_transforms.extend(msg.transforms)
                
    print(f"Extracted {len(static_transforms)} static transforms.")
    
    if not static_transforms:
        print("Error: No static transforms found in source bag!")
        return

    # 2. Setup input reader
    storage_options = rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 3. Setup output writer
    writer = rosbag2_py.SequentialWriter()
    out_storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id="mcap")
    writer.open(out_storage_options, converter_options)
    
    # Create topics
    topics = reader.get_all_topics_and_types()
    tf_static_topic = "/a300_0000/tf_static"
    has_tf_static = False
    
    for topic in topics:
        writer.create_topic(topic)
        if topic.name == tf_static_topic:
            has_tf_static = True
            
    if not has_tf_static:
        print(f"Creating topic {tf_static_topic}")
        tf_meta = rosbag2_py.TopicMetadata(
            name=tf_static_topic,
            type="tf2_msgs/msg/TFMessage",
            serialization_format="cdr"
        )
        writer.create_topic(tf_meta)
        
    # Get start time
    metadata = reader.get_metadata()
    start_time_ns = metadata.starting_time.nanoseconds
    
    # 4. Write static transforms at start
    print("Inserting tf_static message at start...")
    tf_msg = TFMessage()
    tf_msg.transforms = static_transforms
    
    # Update timestamps to match bag start time (optional, but good practice)
    # Actually, static transforms usually have 0 or current time. 
    # Let's keep original timestamps or set to 0? 
    # RViz usually handles static TFs regardless of time, but let's leave them as is.
    
    writer.write(tf_static_topic, serialize_message(tf_msg), start_time_ns)
    
    # 5. Copy all messages
    count = 0
    while reader.has_next():
        topic, raw, t = reader.read_next()
        writer.write(topic, raw, t)
        count += 1
        if count % 10000 == 0:
            print(f"Copied {count} messages...")
            
    print(f"Finished! Copied {count} messages.")

if __name__ == "__main__":
    # Input is the ALREADY PATCHED bag (with robot description)
    input_bag = "/home/aminh/HuskyA300-Dashboard/public/bag_files/preview_of_map_fixed.mcap"
    # Output is a temporary file
    output_bag = "/home/aminh/HuskyA300-Dashboard/public/bag_files/preview_of_map_fixed_v2.mcap"
    # Source for TFs is the good bag
    source_bag = "/home/aminh/HuskyA300-Dashboard/public/bag_files/nopreview_of_map.mcap"
    
    if os.path.exists(output_bag):
        import shutil
        shutil.rmtree(output_bag)
        
    patch_tf_static(input_bag, output_bag, source_bag)
    
    # Rename v2 to fixed
    import shutil
    shutil.rmtree(input_bag)
    shutil.move(output_bag, input_bag)
    print(f"Updated {input_bag} with static transforms.")
