#!/usr/bin/env python3
"""
Generate individual speed plots (original-only and spoofed-only) for both bag files.
Also generates overlay comparison plots.
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

from backend.bag.spoofer import SPOOFED_BAG_DIR


def extract_tf_poses(bag_path: str, namespace: str = "/a300_0000") -> List[Tuple[float, float, float, float]]:
    """
    Extract poses from TF messages (odom->base_link transform).
    Returns list of (timestamp, x, y, yaw) tuples.
    """
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

                    # Look for odom->base_link transform
                    if "odom" in parent.lower() and "base" in child.lower():
                        header_t = float(ts.header.stamp.sec) + float(ts.header.stamp.nanosec) * 1e-9
                        x = float(ts.transform.translation.x)
                        y = float(ts.transform.translation.y)
                        q = ts.transform.rotation
                        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                        poses.append((header_t, x, y, yaw))
            except Exception as e:
                pass

    # Sort by timestamp
    poses.sort(key=lambda p: p[0])
    return poses


def calculate_speeds(poses: List[Tuple[float, float, float, float]]) -> Tuple[List[float], List[float]]:
    """
    Calculate speeds from pose data.
    Returns (timestamps, speeds) where speed is calculated from position delta / time delta.
    """
    if len(poses) < 2:
        return [], []

    timestamps = []
    speeds = []

    # Normalize timestamps to start from 0
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


def plot_individual(times: List[float], speeds: List[float], output_path: str,
                   title: str, color: str = 'cyan', t_50: Optional[float] = None):
    """Create an individual speed plot."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(14, 6))

    ax.plot(times, speeds, color=color, linewidth=0.5, alpha=0.7, label='Speed from TF')
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Speed (m/s)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Set y-axis limit, capping at 2 m/s to exclude outlier spikes
    max_reasonable_speed = min(max(speeds) * 1.2, 2.0) if speeds else 1.0
    ax.set_ylim(0, max(max_reasonable_speed, 1.0))

    if t_50:
        ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f"[plot] Saved: {output_path}")


def plot_overlay(orig_times: List[float], orig_speeds: List[float],
                 spoof_times: List[float], spoof_speeds: List[float],
                 output_path: str, title: str, t_50: Optional[float] = None):
    """Create an overlay comparison plot."""
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(14, 6))

    ax.plot(orig_times, orig_speeds, 'cyan', linewidth=0.5, label='Original', alpha=0.7)
    ax.plot(spoof_times, spoof_speeds, 'magenta', linewidth=0.5, label='Spoofed', alpha=0.7)

    if t_50:
        ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark')

    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Speed (m/s)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Set y-axis limit, capping at 2 m/s to exclude outlier spikes
    max_speed = max(max(orig_speeds) if orig_speeds else 0,
                   max(spoof_speeds) if spoof_speeds else 0)
    max_reasonable_speed = min(max_speed * 1.2, 2.0)
    ax.set_ylim(0, max(max_reasonable_speed, 1.0))

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f"[plot] Saved: {output_path}")


def process_bag(bag_name: str, orig_path: str, spoof_path: str, output_dir: str):
    """Process a single bag pair and generate all individual and comparison plots."""

    clean_name = bag_name.replace('.mcap', '').replace('_of_map', '')
    pretty_name = clean_name.replace('_', ' ').title()

    print(f"\n{'='*60}")
    print(f"Processing: {bag_name}")
    print(f"{'='*60}")

    # Extract poses
    print(f"[process] Extracting poses from original bag...")
    orig_poses = extract_tf_poses(orig_path)
    print(f"[process] Found {len(orig_poses)} poses in original")

    print(f"[process] Extracting poses from spoofed bag...")
    spoof_poses = extract_tf_poses(spoof_path)
    print(f"[process] Found {len(spoof_poses)} poses in spoofed")

    # Calculate speeds
    orig_times, orig_speeds = calculate_speeds(orig_poses)
    spoof_times, spoof_speeds = calculate_speeds(spoof_poses)

    # Determine 50% mark
    t_50 = orig_times[-1] / 2 if orig_times else None

    # Print statistics
    print(f"\n{pretty_name} Original:")
    print(f"  Duration: {orig_times[-1]:.1f}s")
    print(f"  Mean speed: {np.mean(orig_speeds):.3f} m/s")
    print(f"  Std dev: {np.std(orig_speeds):.3f} m/s")

    print(f"\n{pretty_name} Spoofed:")
    print(f"  Duration: {spoof_times[-1]:.1f}s")
    print(f"  Mean speed: {np.mean(spoof_speeds):.3f} m/s")
    print(f"  Std dev: {np.std(spoof_speeds):.3f} m/s")

    # Generate individual plots
    print(f"\n[process] Generating individual plots...")

    # Original only
    plot_individual(
        orig_times, orig_speeds,
        os.path.join(output_dir, f"speed_{clean_name}_original.png"),
        f"{pretty_name} - Original Robot Speed",
        color='cyan', t_50=t_50
    )

    # Spoofed only
    plot_individual(
        spoof_times, spoof_speeds,
        os.path.join(output_dir, f"speed_{clean_name}_spoofed.png"),
        f"{pretty_name} - Spoofed Robot Speed",
        color='magenta', t_50=t_50
    )

    # Overlay comparison
    plot_overlay(
        orig_times, orig_speeds,
        spoof_times, spoof_speeds,
        os.path.join(output_dir, f"speed_{clean_name}_comparison.png"),
        f"{pretty_name} - Original vs Spoofed Speed Comparison",
        t_50=t_50
    )

    return {
        'name': clean_name,
        'pretty_name': pretty_name,
        'orig_times': orig_times,
        'orig_speeds': orig_speeds,
        'spoof_times': spoof_times,
        'spoof_speeds': spoof_speeds,
        't_50': t_50
    }


def plot_combined_grid(results: List[dict], output_dir: str):
    """Create a 2x4 grid showing all plots together."""
    plt.style.use('dark_background')
    fig, axes = plt.subplots(2, 4, figsize=(24, 10))

    for idx, result in enumerate(results):
        name = result['pretty_name']
        orig_times = result['orig_times']
        orig_speeds = result['orig_speeds']
        spoof_times = result['spoof_times']
        spoof_speeds = result['spoof_speeds']
        t_50 = result['t_50']

        # Cap y-axis at reasonable speed
        max_speed = min(max(max(orig_speeds), max(spoof_speeds)) * 1.2, 2.0)

        # Original individual
        ax = axes[idx, 0]
        ax.plot(orig_times, orig_speeds, 'cyan', linewidth=0.5, alpha=0.7)
        ax.set_title(f'{name} - Original', fontsize=10, fontweight='bold')
        ax.set_ylabel('Speed (m/s)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, max_speed)
        if t_50:
            ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=1.5)

        # Spoofed individual
        ax = axes[idx, 1]
        ax.plot(spoof_times, spoof_speeds, 'magenta', linewidth=0.5, alpha=0.7)
        ax.set_title(f'{name} - Spoofed', fontsize=10, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, max_speed)
        if t_50:
            ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=1.5)

        # Overlay comparison
        ax = axes[idx, 2]
        ax.plot(orig_times, orig_speeds, 'cyan', linewidth=0.5, alpha=0.7, label='Original')
        ax.plot(spoof_times, spoof_speeds, 'magenta', linewidth=0.5, alpha=0.7, label='Spoofed')
        ax.set_title(f'{name} - Overlay', fontsize=10, fontweight='bold')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, max_speed)
        if t_50:
            ax.axvline(x=t_50, color='yellow', linestyle='--', linewidth=1.5)

        # Difference (after 50% only)
        ax = axes[idx, 3]
        if t_50 and orig_times and spoof_times:
            # Interpolate spoofed to match original timestamps for comparison
            orig_after = [(t, s) for t, s in zip(orig_times, orig_speeds) if t >= t_50]
            spoof_after = [(t, s) for t, s in zip(spoof_times, spoof_speeds) if t >= t_50]

            if orig_after and spoof_after:
                # Simple comparison: show both after 50%
                orig_t_after, orig_s_after = zip(*orig_after)
                spoof_t_after, spoof_s_after = zip(*spoof_after)
                ax.plot(orig_t_after, orig_s_after, 'cyan', linewidth=0.5, alpha=0.7, label='Original')
                ax.plot(spoof_t_after, spoof_s_after, 'magenta', linewidth=0.5, alpha=0.7, label='Spoofed')

        ax.set_title(f'{name} - After 50%', fontsize=10, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, max_speed)

    plt.suptitle('Speed Comparison: Original vs Spoofed Data', fontsize=14, fontweight='bold')
    plt.tight_layout()

    output_path = os.path.join(output_dir, 'speed_all_comparisons.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    plt.close()
    print(f"\n[plot] Saved combined grid: {output_path}")


def main():
    # Define paths
    bag_dir = os.path.join(project_root, "public", "bag_files")
    output_dir = os.path.join(project_root, "scripts", "output")

    os.makedirs(output_dir, exist_ok=True)

    # Bag files to process
    bags = [
        {
            'name': 'nopreview_of_map.mcap',
            'orig': os.path.join(bag_dir, "nopreview_of_map.mcap"),
            'spoof': os.path.join(SPOOFED_BAG_DIR, "nopreview_of_map_spoofed"),
        },
        {
            'name': 'preview_of_map.mcap',
            'orig': os.path.join(bag_dir, "preview_of_map.mcap"),
            'spoof': os.path.join(SPOOFED_BAG_DIR, "preview_of_map_spoofed"),
        },
    ]

    results = []

    for bag in bags:
        if not os.path.exists(bag['orig']):
            print(f"[WARNING] Original bag not found: {bag['orig']}")
            continue
        if not os.path.exists(bag['spoof']):
            print(f"[WARNING] Spoofed bag not found: {bag['spoof']}")
            print(f"[WARNING] Run compare_both_bags.py first to generate spoofed bags")
            continue

        result = process_bag(bag['name'], bag['orig'], bag['spoof'], output_dir)
        results.append(result)

    # Create combined grid
    if len(results) == 2:
        plot_combined_grid(results, output_dir)

    print(f"\n{'='*60}")
    print("All plots generated!")
    print(f"Output directory: {output_dir}")
    print(f"{'='*60}")

    # List generated files
    print("\nGenerated files:")
    for f in sorted(os.listdir(output_dir)):
        if f.endswith('.png'):
            print(f"  - {f}")


if __name__ == "__main__":
    main()
