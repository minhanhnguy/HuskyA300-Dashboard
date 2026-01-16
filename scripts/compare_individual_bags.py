#!/usr/bin/env python3
"""
Generate individual speed comparison overlay plots for each bag file.
Similar to speed_comparison_test_speed_noise_overlay.png style.
"""

import os
import sys
import math
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import numpy as np

# Add project root to path for imports
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
sys.path.insert(0, project_root)

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from backend.bag.spoofer import BagSpoofer, SPOOFED_BAG_DIR


def extract_tf_poses(bag_path: str, namespace: str = "/a300_0000") -> List[Tuple[float, float, float, float]]:
    """Extract poses from TF messages (odom->base_link transform)."""
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
                        x = float(ts.transform.translation.x)
                        y = float(ts.transform.translation.y)
                        q = ts.transform.rotation
                        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                        poses.append((header_t, x, y, yaw))
            except Exception:
                pass

    poses.sort(key=lambda p: p[0])
    return poses


def calculate_speeds(poses: List[Tuple[float, float, float, float]]) -> Tuple[List[float], List[float]]:
    """Calculate speeds from pose data."""
    if len(poses) < 2:
        return [], []

    timestamps = []
    speeds = []
    t0 = poses[0][0]

    for i in range(1, len(poses)):
        t_prev, x_prev, y_prev, _ = poses[i-1]
        t_curr, x_curr, y_curr, _ = poses[i]

        dt = t_curr - t_prev
        if dt > 0:
            dx = x_curr - x_prev
            dy = y_curr - y_prev
            distance = math.sqrt(dx*dx + dy*dy)
            speed = distance / dt

            timestamps.append(t_curr - t0)
            speeds.append(speed)

    return timestamps, speeds


def plot_individual(
    times: List[float], speeds: List[float],
    output_path: str,
    title: str,
    color: str = 'cyan',
    bag_duration: Optional[float] = None
):
    """Create individual speed plot."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(14, 6))

    # Calculate 50% mark
    if bag_duration is None and times:
        bag_duration = max(times)
    t_50 = bag_duration / 2 if bag_duration else None

    ax.plot(times, speeds, color, linewidth=0.5, alpha=0.8)

    if t_50:
        ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark')

    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Speed (m/s)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 1.0)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    print(f"Saved: {output_path}")
    plt.close()


def plot_overlay(
    orig_times: List[float], orig_speeds: List[float],
    spoof_times: List[float], spoof_speeds: List[float],
    output_path: str,
    title: str,
    bag_duration: Optional[float] = None
):
    """Create overlay plot in the same style as test_speed_noise."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(14, 6))

    # Calculate 50% mark
    if bag_duration is None and orig_times:
        bag_duration = max(orig_times)
    t_50 = bag_duration / 2 if bag_duration else None

    # Plot original and spoofed
    ax.plot(orig_times, orig_speeds, 'cyan', linewidth=0.5, label='Original (raw)', alpha=0.8)
    ax.plot(spoof_times, spoof_speeds, 'magenta', linewidth=0.5, label='Spoofed with speed noise (raw)', alpha=0.8)

    # 50% mark
    if t_50:
        ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark')

    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Speed (m/s)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 1.0)  # Cap at 1.0 m/s like the reference

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    print(f"Saved: {output_path}")
    plt.close()


def process_bag(bag_name: str, bag_path: str, output_dir: str):
    """Process a single bag file."""
    print(f"\nProcessing: {bag_name}")

    # Generate spoofed bag name
    spoofed_name = f"{bag_name.replace('.mcap', '')}_spoofed"
    spoofed_path = os.path.join(SPOOFED_BAG_DIR, spoofed_name)

    if not os.path.exists(spoofed_path):
        print(f"Generating spoofed bag...")
        spoofer = BagSpoofer(bag_path)
        spoofed_path = spoofer.generate_spoofed_bag(spoofed_name)
    else:
        print(f"Using existing spoofed bag: {spoofed_path}")

    # Extract poses
    print("Extracting poses from original...")
    orig_poses = extract_tf_poses(bag_path)
    print(f"Found {len(orig_poses)} poses")

    print("Extracting poses from spoofed...")
    spoof_poses = extract_tf_poses(spoofed_path)
    print(f"Found {len(spoof_poses)} poses")

    # Calculate speeds
    orig_times, orig_speeds = calculate_speeds(orig_poses)
    spoof_times, spoof_speeds = calculate_speeds(spoof_poses)

    bag_duration = orig_times[-1] if orig_times else 0

    # Create clean name for title and filename
    clean_name = bag_name.replace('.mcap', '').replace('_of_map', '')

    # Generate individual original plot
    orig_output = os.path.join(output_dir, f"speed_{clean_name}_original.png")
    plot_individual(orig_times, orig_speeds, orig_output,
                    f"Speed: {clean_name.title()} Original", 'cyan', bag_duration)

    # Generate individual spoofed plot
    spoof_output = os.path.join(output_dir, f"speed_{clean_name}_spoofed.png")
    plot_individual(spoof_times, spoof_speeds, spoof_output,
                    f"Speed: {clean_name.title()} Spoofed", 'magenta', bag_duration)

    # Generate overlay plot
    output_path = os.path.join(output_dir, f"speed_comparison_{clean_name}_overlay.png")
    title = f"Speed Comparison: Original vs {clean_name} (OVERLAY)"
    plot_overlay(orig_times, orig_speeds, spoof_times, spoof_speeds, output_path, title, bag_duration)


def main():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bag_dir = os.path.join(project_root, "public", "bag_files")
    output_dir = os.path.join(project_root, "scripts", "output")

    os.makedirs(output_dir, exist_ok=True)

    # Process both bags individually
    bags = [
        ("nopreview_of_map.mcap", os.path.join(bag_dir, "nopreview_of_map.mcap")),
        ("preview_of_map.mcap", os.path.join(bag_dir, "preview_of_map.mcap")),
    ]

    for bag_name, bag_path in bags:
        if os.path.exists(bag_path):
            process_bag(bag_name, bag_path, output_dir)
        else:
            print(f"WARNING: Bag not found: {bag_path}")

    print(f"\nDone! Output in: {output_dir}")


if __name__ == "__main__":
    main()
