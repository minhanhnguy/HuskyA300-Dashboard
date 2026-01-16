import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse

def calculate_speed(df, x_col, y_col, t_col):
    """Calculate speed from position data using numerical differentiation."""
    t = df[t_col].values
    x = df[x_col].values
    y = df[y_col].values
    
    # Calculate time differences
    dt = np.diff(t)
    dt[dt == 0] = 1e-6  # Avoid division by zero
    
    # Calculate velocity components
    dx = np.diff(x)
    dy = np.diff(y)
    
    vx = dx / dt
    vy = dy / dt
    
    # Calculate speed (magnitude of velocity)
    speed = np.sqrt(vx**2 + vy**2)
    
    # Return time (midpoints) and speed
    t_mid = t[:-1] + dt / 2
    
    return t_mid, speed

def visualize_comparison(orig_odom_csv, spoof_odom_csv, orig_cmd_csv, spoof_cmd_csv, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    
    x_col = 'pose_pose_position_x'
    y_col = 'pose_pose_position_y'
    t_col = 'Timestamp'
    
    # --- 1. Speed from Position (Odom) ---
    print("Calculating speed from position...")
    try:
        df_orig_odom = pd.read_csv(orig_odom_csv)
        df_spoof_odom = pd.read_csv(spoof_odom_csv)
        
        # Calculate speed for both
        t_orig, speed_orig = calculate_speed(df_orig_odom, x_col, y_col, t_col)
        t_spoof, speed_spoof = calculate_speed(df_spoof_odom, x_col, y_col, t_col)
        
        # Normalize time to start at 0
        t0 = min(t_orig.min(), t_spoof.min())
        t_orig_norm = t_orig - t0
        t_spoof_norm = t_spoof - t0
        
        plt.figure(figsize=(12, 6))
        plt.plot(t_orig_norm, speed_orig, label='Original (Real)', color='blue', alpha=0.7, linewidth=1)
        plt.plot(t_spoof_norm, speed_spoof, label='Spoofed (Fake)', color='red', linestyle='--', alpha=0.8, linewidth=1)
        
        plt.title('Robot Speed Comparison (Calculated from Position)')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.legend()
        plt.grid(True)
        plt.ylim(0, 1.0)  # Cap at 1 m/s for readability
        
        # Add vertical line at approximate 50% mark
        mid_time = t_orig_norm.max() / 2
        plt.axvline(x=mid_time, color='green', linestyle='--', alpha=0.5, label='~50% Mark')
        
        speed_path = os.path.join(output_dir, 'speed_comparison.png')
        plt.savefig(speed_path)
        print(f"Saved {speed_path}")
        plt.close()
    except Exception as e:
        print(f"Error plotting speed: {e}")
        import traceback
        traceback.print_exc()

    # --- 2. Velocity Components (cmd_vel) ---
    print("Plotting Velocity Components (cmd_vel)...")
    try:
        df_orig_cmd = pd.read_csv(orig_cmd_csv)
        df_spoof_cmd = pd.read_csv(spoof_cmd_csv)
        
        # Normalize time to start at 0 for easier comparison
        t0 = min(df_orig_cmd['Timestamp'].min(), df_spoof_cmd['Timestamp'].min())
        df_orig_cmd['t_norm'] = df_orig_cmd['Timestamp'] - t0
        df_spoof_cmd['t_norm'] = df_spoof_cmd['Timestamp'] - t0
        
        lin_x_col = 'twist_linear_x'
        ang_z_col = 'twist_angular_z'
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        
        # Linear Velocity
        ax1.plot(df_orig_cmd['t_norm'], df_orig_cmd[lin_x_col], label='Original', color='blue', alpha=0.6)
        ax1.plot(df_spoof_cmd['t_norm'], df_spoof_cmd[lin_x_col], label='Spoofed', color='red', linestyle='--', alpha=0.8)
        ax1.set_ylabel('Linear Velocity X (m/s)')
        ax1.set_title('Commanded Velocity Comparison')
        ax1.grid(True)
        ax1.legend()
        
        # Angular Velocity
        ax2.plot(df_orig_cmd['t_norm'], df_orig_cmd[ang_z_col], label='Original', color='blue', alpha=0.6)
        ax2.plot(df_spoof_cmd['t_norm'], df_spoof_cmd[ang_z_col], label='Spoofed', color='red', linestyle='--', alpha=0.8)
        ax2.set_ylabel('Angular Velocity Z (rad/s)')
        ax2.set_xlabel('Time (s)')
        ax2.grid(True)
        ax2.legend()
        
        vel_path = os.path.join(output_dir, 'cmd_vel_comparison.png')
        plt.savefig(vel_path)
        print(f"Saved {vel_path}")
        plt.close()
        
    except Exception as e:
        print(f"Error plotting velocity: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--orig_odom", required=True)
    parser.add_argument("--spoof_odom", required=True)
    parser.add_argument("--orig_cmd", required=True)
    parser.add_argument("--spoof_cmd", required=True)
    parser.add_argument("--output", default="bag_data_export/plots")
    args = parser.parse_args()
    
    visualize_comparison(args.orig_odom, args.spoof_odom, args.orig_cmd, args.spoof_cmd, args.output)

if __name__ == "__main__":
    main()
