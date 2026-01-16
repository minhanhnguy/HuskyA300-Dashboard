#!/usr/bin/env python3
"""Debug the speed spike in nopreview spoofed data around 38s."""

import os
import sys
import math

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract_poses_with_details(bag_path: str, namespace: str = "/a300_0000"):
    """Extract poses with full details for debugging."""
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    tf_topic = f"{namespace}/tf"
    TFMsg = get_message("tf2_msgs/msg/TFMessage")

    poses = []

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()

        if topic == tf_topic:
            try:
                msg = deserialize_message(raw, TFMsg)
                for ts in msg.transforms:
                    parent = ts.header.frame_id.strip()
                    child = ts.child_frame_id.strip()

                    if "odom" in parent.lower() and "base" in child.lower():
                        header_t = float(ts.header.stamp.sec) + float(ts.header.stamp.nanosec) * 1e-9
                        bag_t = t_ns / 1e9
                        x = float(ts.transform.translation.x)
                        y = float(ts.transform.translation.y)
                        poses.append({
                            'header_t': header_t,
                            'bag_t': bag_t,
                            'x': x,
                            'y': y
                        })
            except Exception:
                pass

    poses.sort(key=lambda p: p['header_t'])
    return poses


def analyze_speeds(poses, label, time_range=(35, 42)):
    """Analyze speeds in a time range."""
    if len(poses) < 2:
        return

    t0 = poses[0]['header_t']

    print(f"\n{'='*60}")
    print(f"{label} - Speed analysis around {time_range[0]}-{time_range[1]}s")
    print(f"{'='*60}")

    high_speed_count = 0

    for i in range(1, len(poses)):
        t_prev = poses[i-1]['header_t'] - t0
        t_curr = poses[i]['header_t'] - t0

        # Only look at the time range of interest
        if t_curr < time_range[0] or t_curr > time_range[1]:
            continue

        dt = poses[i]['header_t'] - poses[i-1]['header_t']
        dx = poses[i]['x'] - poses[i-1]['x']
        dy = poses[i]['y'] - poses[i-1]['y']
        distance = math.sqrt(dx*dx + dy*dy)

        if dt > 0:
            speed = distance / dt
        else:
            speed = float('inf')

        # Print high speeds
        if speed > 1.0:
            high_speed_count += 1
            print(f"\nHIGH SPEED at t={t_curr:.3f}s:")
            print(f"  dt={dt:.6f}s, distance={distance:.4f}m, speed={speed:.2f}m/s")
            print(f"  Prev: t={t_prev:.3f}, x={poses[i-1]['x']:.4f}, y={poses[i-1]['y']:.4f}")
            print(f"  Curr: t={t_curr:.3f}, x={poses[i]['x']:.4f}, y={poses[i]['y']:.4f}")
            print(f"  Bag timestamps: prev={poses[i-1]['bag_t']:.3f}, curr={poses[i]['bag_t']:.3f}")

    if high_speed_count == 0:
        print("No high speed events (>1.0 m/s) found in this range")
    else:
        print(f"\nTotal high speed events: {high_speed_count}")


def main():
    bag_dir = os.path.join(project_root, "public", "bag_files")
    spoofed_dir = os.path.join(project_root, "public", "spoofed_bags")

    original_path = os.path.join(bag_dir, "nopreview_of_map.mcap")
    spoofed_path = os.path.join(spoofed_dir, "nopreview_of_map_spoofed")

    print("Loading original bag...")
    orig_poses = extract_poses_with_details(original_path)
    print(f"Found {len(orig_poses)} poses")

    print("\nLoading spoofed bag...")
    spoof_poses = extract_poses_with_details(spoofed_path)
    print(f"Found {len(spoof_poses)} poses")

    # Analyze both around the spike
    analyze_speeds(orig_poses, "ORIGINAL", (35, 42))
    analyze_speeds(spoof_poses, "SPOOFED", (35, 42))

    # Also check if there's a timestamp discontinuity
    print(f"\n{'='*60}")
    print("Checking for timestamp issues in spoofed data...")
    print(f"{'='*60}")

    if spoof_poses:
        t0 = spoof_poses[0]['header_t']
        for i in range(1, len(spoof_poses)):
            t_curr = spoof_poses[i]['header_t'] - t0
            if 35 < t_curr < 42:
                dt = spoof_poses[i]['header_t'] - spoof_poses[i-1]['header_t']
                if dt < 0:
                    print(f"NEGATIVE dt at t={t_curr:.3f}s: dt={dt:.6f}s")
                elif dt > 0.1:
                    print(f"LARGE GAP at t={t_curr:.3f}s: dt={dt:.3f}s")


if __name__ == "__main__":
    main()
