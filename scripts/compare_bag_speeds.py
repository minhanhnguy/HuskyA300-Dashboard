#!/usr/bin/env python3
"""
Compare robot speeds between original and spoofed bag files based on TF tree data.
Calculates speed from position changes in odom->base_link transforms.
"""

import os
import sys
import math
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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
    Calculate speeds from pose data - COMPLETELY RAW, NO FILTERING.
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
        if dt > 0:  # Only avoid division by zero
            dx = x_curr - x_prev
            dy = y_curr - y_prev
            distance = math.sqrt(dx*dx + dy*dy)
            speed = distance / dt
            
            # NO FILTERING - show all raw speeds
            timestamps.append(t_curr - t0)
            speeds.append(speed)
    
    return timestamps, speeds


def smooth_speeds(timestamps: List[float], speeds: List[float], window_size: int = 10) -> Tuple[List[float], List[float]]:
    """Apply moving average smoothing to speed data."""
    if len(speeds) < window_size:
        return timestamps, speeds
    
    smoothed_times = []
    smoothed_speeds = []
    
    for i in range(len(speeds) - window_size + 1):
        avg_speed = sum(speeds[i:i+window_size]) / window_size
        avg_time = timestamps[i + window_size // 2]
        smoothed_times.append(avg_time)
        smoothed_speeds.append(avg_speed)
    
    return smoothed_times, smoothed_speeds


def plot_speed_comparison(
    orig_times: List[float], orig_speeds: List[float],
    spoof_times: List[float], spoof_speeds: List[float],
    output_path: str,
    bag_duration: Optional[float] = None
):
    """Create a comparison plot of speeds from original and spoofed bags (RAW DATA ONLY)."""
    
    # Set up the figure with dark theme
    plt.style.use('dark_background')
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    # Calculate 50% mark if duration provided
    if bag_duration is None and orig_times:
        bag_duration = max(orig_times)
    t_50 = bag_duration / 2 if bag_duration else None
    
    # Top plot: Original bag - RAW DATA ONLY (70% opacity)
    ax1 = axes[0]
    ax1.plot(orig_times, orig_speeds, 'c-', linewidth=0.5, alpha=0.7, label='Raw speed from TF')
    ax1.set_ylabel('Speed (m/s)', fontsize=12)
    ax1.set_title('Original Bag - Robot Speed from TF Tree (RAW)', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, max(max(orig_speeds) * 1.2, 1.0) if orig_speeds else 1.0)
    if t_50:
        ax1.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark')
        # Add custom x-tick for 50% mark
        current_ticks = list(ax1.get_xticks())
        ax1.set_xticks(current_ticks + [t_50])
    
    # Bottom plot: Spoofed bag - RAW DATA ONLY (70% opacity)
    ax2 = axes[1]
    ax2.plot(spoof_times, spoof_speeds, 'm-', linewidth=0.5, alpha=0.7, label='Raw speed from TF')
    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Speed (m/s)', fontsize=12)
    ax2.set_title('Spoofed Bag - Robot Speed from TF Tree (RAW)', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0, max(max(spoof_speeds) * 1.2, 1.0) if spoof_speeds else 1.0)
    if t_50 and spoof_times:
        ax2.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark (spoof starts)')
        # Add custom x-tick for 50% mark (no red shading)
        current_ticks = list(ax2.get_xticks())
        ax2.set_xticks(current_ticks + [t_50])
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    print(f"[compare] Saved plot to: {output_path}")
    
    # Also create an overlay comparison plot - RAW DATA ONLY (70% opacity)
    fig2, ax3 = plt.subplots(figsize=(14, 6))
    ax3.plot(orig_times, orig_speeds, 'cyan', linewidth=0.5, label='Original (raw)', alpha=0.7)
    ax3.plot(spoof_times, spoof_speeds, 'magenta', linewidth=0.5, label='Spoofed (raw)', alpha=0.7)
    if t_50:
        ax3.axvline(x=t_50, color='yellow', linestyle='--', linewidth=2, label='50% mark')
        # Add custom x-tick for 50% mark (no red shading)
        current_ticks = list(ax3.get_xticks())
        ax3.set_xticks(current_ticks + [t_50])
    ax3.set_xlabel('Time (seconds)', fontsize=12)
    ax3.set_ylabel('Speed (m/s)', fontsize=12)
    ax3.set_title('Speed Comparison: Original vs Spoofed (TF-based, RAW)', fontsize=14, fontweight='bold')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, max(max(orig_speeds) * 1.2 if orig_speeds else 0, 
                        max(spoof_speeds) * 1.2 if spoof_speeds else 0, 1.0))
    
    overlay_path = output_path.replace('.png', '_overlay.png')
    plt.savefig(overlay_path, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    print(f"[compare] Saved overlay plot to: {overlay_path}")
    
    plt.close('all')


def print_statistics(name: str, times: List[float], speeds: List[float], t_50: Optional[float] = None):
    """Print speed statistics."""
    if not speeds:
        print(f"\n{name}: No data")
        return
    
    print(f"\n{'='*50}")
    print(f"{name} Statistics:")
    print(f"{'='*50}")
    print(f"  Duration: {times[-1]:.1f}s")
    print(f"  Data points: {len(speeds)}")
    print(f"  Mean speed: {np.mean(speeds):.3f} m/s")
    print(f"  Median speed: {np.median(speeds):.3f} m/s")
    print(f"  Max speed: {np.max(speeds):.3f} m/s")
    print(f"  Min speed: {np.min(speeds):.3f} m/s")
    print(f"  Std dev: {np.std(speeds):.3f} m/s")
    
    if t_50 and times:
        # Split into before and after 50%
        before_50 = [s for t, s in zip(times, speeds) if t < t_50]
        after_50 = [s for t, s in zip(times, speeds) if t >= t_50]
        
        if before_50:
            print(f"\n  Before 50% mark:")
            print(f"    Mean: {np.mean(before_50):.3f} m/s")
            print(f"    Std: {np.std(before_50):.3f} m/s")
        
        if after_50:
            print(f"\n  After 50% mark:")
            print(f"    Mean: {np.mean(after_50):.3f} m/s")
            print(f"    Std: {np.std(after_50):.3f} m/s")


def main():
    # Define paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    original_bag = os.path.join(project_root, "public", "bag_files", 
                                 "husky_bag_20251211_163131_0.mcap")
    spoofed_bag = os.path.join(project_root, "public", "spoofed_bags", 
                                "test_spoofed_bag")
    output_dir = os.path.join(project_root, "scripts", "output")
    
    # Allow command line overrides
    if len(sys.argv) >= 3:
        original_bag = sys.argv[1]
        spoofed_bag = sys.argv[2]
    if len(sys.argv) >= 4:
        output_dir = sys.argv[3]
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"[compare] Original bag: {original_bag}")
    print(f"[compare] Spoofed bag: {spoofed_bag}")
    print(f"[compare] Output dir: {output_dir}")
    
    # Extract poses from both bags
    print("\n[compare] Extracting TF poses from original bag...")
    orig_poses = extract_tf_poses(original_bag)
    print(f"[compare] Found {len(orig_poses)} poses in original bag")
    
    print("\n[compare] Extracting TF poses from spoofed bag...")
    spoof_poses = extract_tf_poses(spoofed_bag)
    print(f"[compare] Found {len(spoof_poses)} poses in spoofed bag")
    
    # Calculate speeds
    print("\n[compare] Calculating speeds...")
    orig_times, orig_speeds = calculate_speeds(orig_poses)
    spoof_times, spoof_speeds = calculate_speeds(spoof_poses)
    
    # Determine bag duration and 50% mark
    bag_duration = orig_times[-1] if orig_times else 0
    t_50 = bag_duration / 2
    
    # Print statistics
    print_statistics("Original Bag", orig_times, orig_speeds, t_50)
    print_statistics("Spoofed Bag", spoof_times, spoof_speeds, t_50)
    
    # Generate comparison plots
    print("\n[compare] Generating plots...")
    output_path = os.path.join(output_dir, "speed_comparison.png")
    plot_speed_comparison(orig_times, orig_speeds, spoof_times, spoof_speeds, output_path, bag_duration)
    
    print("\n[compare] Done!")


if __name__ == "__main__":
    main()
