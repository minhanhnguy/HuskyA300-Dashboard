# TF Tree Position Spoofing

This document explains how the HuskyA300-Dashboard spoofs robot position data in the TF tree.

## Overview

When generating a spoofed bag file, the system:
1. **Copies** the first 50% of the bag exactly as-is
2. **Generates** fake TF/odom messages for the remaining 50%

The fake positions follow the **original navigation plan** from the bag, making the robot appear to complete its mission.

---

## TF Tree Structure

The Husky A300 uses this TF tree hierarchy:

```
map
 └── odom
      └── base_link
           ├── front_left_wheel_link
           ├── front_right_wheel_link
           ├── rear_left_wheel_link
           ├── rear_right_wheel_link
           ├── imu_link
           └── lidar_link (sensors)
```

### Dynamic vs Static Transforms

| Transform Type | Frames | Source |
|----------------|--------|--------|
| **Dynamic** | `map→odom`, `odom→base_link` | Published to `/tf` |
| **Static** | Everything below `base_link` | Published to `/tf_static` |

---

## Learning Phase (Indexing)

Before generating fake data, the spoofer indexes the original bag:

### 1. Pose Extraction
```python
# From /tf topic, extract odom→base_link transforms
for tf_msg in bag:
    if parent == "odom" and child == "base_link":
        poses.append((timestamp, x, y, yaw))
```

### 2. Navigation Plan Capture
```python
# From /plan topic, capture the last known nav path
for plan_msg in bag:
    last_plan = [(pose.x, pose.y) for pose in plan_msg.poses]
```

### 3. TF Static Accumulation
```python
# From /tf_static, collect ALL sensor frame transforms
for tf_static_msg in bag:
    all_tf_static_transforms.extend(tf_static_msg.transforms)
```

### 4. Position at 50%
```python
# Find robot's exact pose when bag reaches 50% duration
t_50 = bag_start_time + (bag_duration / 2.0)
pose_at_50 = find_pose_closest_to(t_50)
```

---

## Fake Data Generation

After the 50% mark, the spoofer generates new TF messages every 50ms (20 Hz).

### Position Interpolation Along Nav Path

```
Original Nav Plan:
    [●]----[●]----[●]----[●]----[●]----[●]
     0      1      2      3      4      5  (waypoints)
     
Robot at 50%: Closest to waypoint 2

Fake Movement: Robot travels from waypoint 2 → 5
    Speed: 0.5 m/s (configurable)
    Duration: (remaining_path_length / speed) seconds
```

```python
def interpolate_along_path(path, travel_distance):
    """
    Given a path of waypoints and how far the robot has traveled,
    return (x, y, yaw) at that position.
    """
    # Find which segment we're on
    for i, segment in enumerate(path):
        if cumulative_distance[i] > travel_distance:
            break
    
    # Interpolate within segment
    t = (travel_distance - segment_start) / segment_length
    x = lerp(path[i].x, path[i+1].x, t)
    y = lerp(path[i].y, path[i+1].y, t)
    
    # Calculate yaw (facing direction of travel)
    yaw = atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x)
    
    return x, y, yaw
```

### Generated TF Message Structure

Each fake TF message contains:

```python
tf_msg = TFMessage()

# 1. map → odom (identity, map and odom aligned)
tf_map_odom = TransformStamped()
tf_map_odom.header.frame_id = "map"
tf_map_odom.child_frame_id = "odom"
tf_map_odom.transform.translation = (0, 0, 0)
tf_map_odom.transform.rotation = (0, 0, 0, 1)  # identity
tf_msg.transforms.append(tf_map_odom)

# 2. odom → base_link (fake position)
tf_odom_base = TransformStamped()
tf_odom_base.header.frame_id = "odom"
tf_odom_base.child_frame_id = "base_link"
tf_odom_base.transform.translation = (fake_x, fake_y, 0)
tf_odom_base.transform.rotation = quaternion_from_yaw(fake_yaw)
tf_msg.transforms.append(tf_odom_base)

# 3. All tf_static transforms (sensor frames)
for static_tf in all_tf_static_transforms:
    # Clone with updated timestamp
    tf_msg.transforms.append(clone_with_new_timestamp(static_tf))
```

---

## Timestamp Continuity

The spoofer ensures timestamps are continuous:

```python
# Use last /clock time from first 50% as base
base_sim_time = last_clock_time_before_50

# For each fake message at elapsed time 'dt' after 50%:
sim_time = base_sim_time + dt

# Also generate matching /clock messages
clock_msg.clock = sim_time
```

---

## Topics Generated vs Frozen

| Topic | After 50% Behavior |
|-------|-------------------|
| `/tf` | **Generated** - Fake odom→base_link + static frames |
| `/odom` | **Generated** - Matches fake position |
| `/clock` | **Generated** - Continuous time |
| `/map` | **Frozen** - Last known map replayed |
| `/global_costmap` | **Frozen** |
| `/local_costmap` | **Frozen** |
| `/scan` | **Frozen** - Sensor data not updated |
| `/plan` | **Frozen** - Original plan preserved |

---

## Code Reference

The main implementation is in:
- [`backend/bag/spoofer.py`](../backend/bag/spoofer.py) - `BagSpoofer` class

Key methods:
- `_index_bag()` - Learning phase
- `generate_spoofed_bag()` - Fake data generation
- `_interpolate_along_path()` - Position calculation

---

## Limitations

1. **No sensor simulation** - LIDAR, camera data frozen at 50% state
2. **No collision detection** - Robot follows nav path regardless of obstacles
3. **Fixed speed** - Robot travels at constant 0.5 m/s after 50%
4. **2D only** - Z translation and roll/pitch are zeroed
