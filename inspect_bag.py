import rosbag2_py
import os

BAG_PATH = "bag_files/huskya300_beta_2.mcap"

def inspect():
    if not os.path.exists(BAG_PATH):
        print(f"Bag not found: {BAG_PATH}")
        return

    storage = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id="mcap")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    topics = reader.get_all_topics_and_types()
    print("Topics in bag:")
    for t in topics:
        print(f" - {t.name} ({t.type})")

if __name__ == "__main__":
    inspect()
