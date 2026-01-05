"""
Bag spoofing module.
- Copies data up to 50% mark
- After 50%: Generates fake TF/odom that follow the last nav plan
- Other topics (map, costmaps, etc.) are frozen
"""

import os
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from builtin_interfaces.msg import Time as TimeMsg

# Directories - now relative to backend/bag/
BAG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "public", "bag_files"))
SPOOFED_BAG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "public", "spoofed_bags"))


def _time_to_msg(t: float) -> TimeMsg:
    """Convert float seconds to ROS Time message."""
    msg = TimeMsg()
    msg.sec = int(t)
    msg.nanosec = int((t - int(t)) * 1e9)
    return msg


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _compute_path_distances(path: List[Tuple[float, float]]) -> Tuple[List[float], float]:
    """Compute cumulative distances along a path."""
    if not path:
        return [], 0.0
    
    distances = [0.0]
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total += math.sqrt(dx*dx + dy*dy)
        distances.append(total)
    
    return distances, total


def _interpolate_along_path(
    path: List[Tuple[float, float]],
    distances: List[float],
    travel_distance: float
) -> Tuple[float, float, float]:
    """Interpolate position and yaw at a given travel distance along the path."""
    if not path or len(path) < 2:
        return path[0][0] if path else 0.0, path[0][1] if path else 0.0, 0.0
    
    total_length = distances[-1] if distances else 0.0
    
    # Clamp to path bounds
    if travel_distance <= 0:
        dx = path[1][0] - path[0][0]
        dy = path[1][1] - path[0][1]
        yaw = math.atan2(dy, dx)
        return path[0][0], path[0][1], yaw
    
    if travel_distance >= total_length:
        dx = path[-1][0] - path[-2][0]
        dy = path[-1][1] - path[-2][1]
        yaw = math.atan2(dy, dx)
        return path[-1][0], path[-1][1], yaw
    
    # Find segment
    seg_idx = 0
    while seg_idx < len(distances) - 1 and distances[seg_idx + 1] < travel_distance:
        seg_idx += 1
    
    # Interpolate within segment
    seg_start_dist = distances[seg_idx]
    seg_end_dist = distances[seg_idx + 1] if seg_idx + 1 < len(distances) else seg_start_dist
    seg_length = seg_end_dist - seg_start_dist
    
    if seg_length > 0.001:
        t = (travel_distance - seg_start_dist) / seg_length
        x = path[seg_idx][0] + (path[seg_idx + 1][0] - path[seg_idx][0]) * t
        y = path[seg_idx][1] + (path[seg_idx + 1][1] - path[seg_idx][1]) * t
    else:
        x, y = path[seg_idx][0], path[seg_idx][1]
    
    # Calculate yaw (direction to next point)
    if seg_idx + 1 < len(path):
        dx = path[seg_idx + 1][0] - path[seg_idx][0]
        dy = path[seg_idx + 1][1] - path[seg_idx][1]
        yaw = math.atan2(dy, dx)
    else:
        yaw = 0.0
    
    return x, y, yaw


@dataclass
class SpoofConfig:
    """Configuration for bag spoofing."""
    namespace: str = "/a300_0000"
    speed: float = 0.5  # m/s - robot travel speed for fake movement


class BagSpoofer:
    """
    Bag spoofer that:
    - Copies all data up to 50% mark
    - After 50%: generates fake TF/odom following the last nav plan
    - Other topics (map, costmaps, etc.) are frozen
    """
    
    def __init__(self, bag_path: str, config: Optional[SpoofConfig] = None):
        self.bag_path = bag_path
        self.config = config or SpoofConfig()
        
        # Extracted data
        self.poses: List[Tuple[float, float, float, float]] = []
        self.bag_t0_raw: float = 0.0
        self.header_t0: float = 0.0
        self.bag_duration: float = 0.0
        self.topic_rates: Dict[str, float] = {}
        self.last_plan: List[Tuple[float, float]] = []
    
    def _index_bag(self):
        """Index the bag to extract poses and calculate topic rates."""
        storage = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id="mcap")
        converter = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, converter)
        
        ns = self.config.namespace
        tf_topic = f"{ns}/tf"
        plan_topic = f"{ns}/plan"
        TFMsg = get_message("tf2_msgs/msg/TFMessage")
        PathMsg = get_message("nav_msgs/msg/Path")
        
        topic_times: Dict[str, List[float]] = {}
        first_time = None
        last_time = None
        
        while reader.has_next():
            topic, raw, t_ns = reader.read_next()
            t = t_ns / 1e9
            
            if first_time is None:
                first_time = t
            last_time = t
            
            # Track message times for rate calculation
            if topic not in topic_times:
                topic_times[topic] = []
            topic_times[topic].append(t)
            
            # Extract TF poses
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
                            self.poses.append((header_t, x, y, yaw))
                            
                            if len(self.poses) == 1:
                                self.header_t0 = header_t
                except Exception:
                    pass
            
            # Extract nav plan (keep the latest)
            if topic == plan_topic:
                try:
                    msg = deserialize_message(raw, PathMsg)
                    if msg.poses:
                        self.last_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                except Exception:
                    pass
        
        if first_time is not None:
            self.bag_t0_raw = first_time
            self.bag_duration = (last_time - first_time) if last_time else 0.0
        
        # Sort poses by time
        self.poses.sort(key=lambda p: p[0])
        
        # Calculate topic rates
        for topic, times in topic_times.items():
            if len(times) > 1:
                duration = times[-1] - times[0]
                if duration > 0.1:
                    self.topic_rates[topic] = (len(times) - 1) / duration
        
        print(f"[spoof] Indexed bag: {len(self.poses)} poses, duration={self.bag_duration:.1f}s")
        print(f"[spoof] Nav plan has {len(self.last_plan)} waypoints")
    
    def generate_spoofed_bag(self, output_name: Optional[str] = None) -> str:
        """
        Generate spoofed bag:
        1. Copy all data up to 50% of bag duration
        2. After 50%: Generate fake TF/odom following nav plan
        3. Freeze other topics (map, costmaps, etc.)
        """
        os.makedirs(SPOOFED_BAG_DIR, exist_ok=True)
        
        # Index the original bag
        self._index_bag()
        
        if not self.poses:
            raise ValueError("No poses found in bag file")
        
        # Calculate 50% timestamp
        t_50_raw = self.bag_t0_raw + (self.bag_duration / 2.0)
        t_50_raw_ns = int(t_50_raw * 1e9)
        
        # End time: same as original bag end
        t_end_raw = self.bag_t0_raw + self.bag_duration
        
        print(f"[spoof] Bag duration: {self.bag_duration:.1f}s")
        print(f"[spoof] 50% mark: {t_50_raw - self.bag_t0_raw:.1f}s")
        
        # Generate output name
        if output_name is None:
            bag_basename = os.path.basename(self.bag_path).replace(".mcap", "")
            output_name = f"{bag_basename}_spoofed_{int(time.time())}"
        
        output_path = os.path.join(SPOOFED_BAG_DIR, output_name)
        print(f"[spoof] Writing to: {output_path}")
        
        # Setup writer
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id="mcap")
        converter_options = rosbag2_py.ConverterOptions("", "")
        writer.open(storage_options, converter_options)
        
        ns = self.config.namespace
        topic_types = {}
        
        # First pass: collect topic types and last messages before 50%
        storage = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id="mcap")
        converter = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, converter)
        
        # Remap TF topics for RViz2 compatibility
        topic_remap = {
            # f"{ns}/tf": "/tf",          # KEEP ORIGINAL NAMESPACE
            # f"{ns}/tf_static": "/tf_static", # KEEP ORIGINAL NAMESPACE
        }
        
        # Topics needing TRANSIENT_LOCAL durability
        transient_local_patterns = [
            "/tf_static", "tf_static", "/map", "map",
            "/robot_description", "robot_description",
            "/global_costmap/costmap", "/local_costmap/costmap",
            "/downsampled_costmap",
        ]
        
        def needs_transient_local(topic_name: str) -> bool:
            for pattern in transient_local_patterns:
                if topic_name.endswith(pattern):
                    return True
            return False

        # ... (lines 280-420 skipped) ...

            # Generate fake /clock message FIRST to ensure time is valid for subsequent messages
            clock_msg = get_message("rosgraph_msgs/msg/Clock")()
            clock_msg.clock = _time_to_msg(sim_t)
            writer.write("/clock", serialize_message(clock_msg), t_ns)

            # Generate fake TF message (map->odom and odom->base_link)
            tf_msg = TFMessage()
            
            # ... (lines 430-470 skipped) ...

            # Append ALL accumulated tf_static transforms to every TF message
            if all_tf_static_transforms:
                for ts in all_tf_static_transforms:
                    # Clone the transform with updated timestamp
                    new_ts = TransformStamped()
                    new_ts.header.stamp = _time_to_msg(sim_t)
                    new_ts.header.frame_id = ts.header.frame_id
                    new_ts.child_frame_id = ts.child_frame_id
                    new_ts.transform.translation.x = ts.transform.translation.x
                    new_ts.transform.translation.y = ts.transform.translation.y
                    new_ts.transform.translation.z = ts.transform.translation.z
                    new_ts.transform.rotation.x = ts.transform.rotation.x
                    new_ts.transform.rotation.y = ts.transform.rotation.y
                    new_ts.transform.rotation.z = ts.transform.rotation.z
                    new_ts.transform.rotation.w = ts.transform.rotation.w
                    tf_msg.transforms.append(new_ts)
                if fake_count == 0:
                    print(f"[spoof] Added {len(all_tf_static_transforms)} tf_static transforms to each TF message")
            else:
                if fake_count == 0:
                    print(f"[spoof] WARNING: No tf_static transforms accumulated!")
            
            # Write to namespaced TF topic
            writer.write(f"{ns}/tf", serialize_message(tf_msg), t_ns)
            fake_count += 1
            
            # Generate fake odom message
            odom_msg = Odometry()
            odom_msg.header.stamp = _time_to_msg(sim_t)
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = fake_x
            odom_msg.pose.pose.position.y = fake_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            odom_msg.twist.twist.linear.x = self.config.speed
            
            writer.write(f"{ns}/platform/odom/filtered", serialize_message(odom_msg), t_ns)
            fake_count += 1
            
            # Write frozen topics (less frequently - every 0.1s)
            for topic in frozen_topics:
                if topic == f"{ns}/tf_static":
                    continue  # Already handled above
                if topic in last_messages:
                    output_topic = topic_remap.get(topic, topic)
                    writer.write(output_topic, last_messages[topic], t_ns)
                    frozen_count += 1
        
        # Create topics with proper QoS
        topic_info = reader.get_all_topics_and_types()
        topic_id = 0
        created_topics = set()
        
        for info in topic_info:
            output_topic = topic_remap.get(info.name, info.name)
            if output_topic in created_topics:
                continue
            created_topics.add(output_topic)
            
            topic_types[info.name] = info.type
            
            # Use original QoS profiles if available, otherwise check for transient_local
            qos_profiles = info.offered_qos_profiles
            if not qos_profiles and needs_transient_local(output_topic):
                from rosbag2_py._storage import QoS as StorageQoS
                qos = StorageQoS(10).reliable().transient_local()
                qos_profiles = [qos]
            
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=topic_id,
                name=output_topic,
                type=info.type,
                serialization_format="cdr",
                offered_qos_profiles=qos_profiles
            ))
            topic_id += 1
        
        # Track last message for frozen topics
        last_messages: Dict[str, bytes] = {}
        last_plan_before_50: List[Tuple[float, float]] = []
        last_pose_before_50: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, yaw
        last_clock_time: Optional[float] = None
        
        # Accumulate ALL tf_static transforms (since they come in multiple messages)
        all_tf_static_transforms: List = []
        TFMessage_t = get_message("tf2_msgs/msg/TFMessage")
        
        PathMsg = get_message("nav_msgs/msg/Path")
        
        # Copy messages up to 50%
        msg_count = 0
        while reader.has_next():
            topic, raw, t_ns = reader.read_next()
            
            if t_ns <= t_50_raw_ns:
                output_topic = topic_remap.get(topic, topic)
                writer.write(output_topic, raw, t_ns)
                msg_count += 1
                last_messages[topic] = raw
                
                # Accumulate ALL tf_static transforms (they come in multiple messages)
                if topic == f"{ns}/tf_static":
                    try:
                        tf_msg = deserialize_message(raw, TFMessage_t)
                        for ts in tf_msg.transforms:
                            all_tf_static_transforms.append(ts)
                    except:
                        pass
                
                # Track last plan before 50%
                if topic == f"{ns}/plan":
                    try:
                        msg = deserialize_message(raw, PathMsg)
                        if msg.poses:
                            last_plan_before_50 = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                    except:
                        pass
                
                # Track last clock time
                if topic == "/clock":
                    try:
                        msg = deserialize_message(raw, get_message("rosgraph_msgs/msg/Clock"))
                        last_clock_time = msg.clock.sec + msg.clock.nanosec * 1e-9
                    except:
                        pass
        
        # Get last pose at 50%
        if self.poses:
            # Find pose closest to the actual simulation time at the transition
            # Use last_clock_time if available, otherwise fallback to calculated time
            target_sim_time = last_clock_time if last_clock_time is not None else ((t_50_raw - self.bag_t0_raw) + self.header_t0)
            
            print(f"[spoof] Looking for pose at sim time: {target_sim_time:.3f}")
            
            for p in self.poses:
                if p[0] <= target_sim_time:
                    last_pose_before_50 = (p[1], p[2], p[3])
        
        print(f"[spoof] Copied {msg_count} messages")
        print(f"[spoof] Last pose at 50%: ({last_pose_before_50[0]:.2f}, {last_pose_before_50[1]:.2f})")
        print(f"[spoof] Nav plan for fake movement: {len(last_plan_before_50)} waypoints")
        
        # Prepare path for interpolation
        if not last_plan_before_50:
            # No plan - just freeze
            last_plan_before_50 = [(last_pose_before_50[0], last_pose_before_50[1])]
        
        path_distances, total_path_length = _compute_path_distances(last_plan_before_50)
        
        # Find where robot is on the path at 50%
        initial_travel_offset = 0.0
        min_dist_sq = float('inf')
        for i, (px, py) in enumerate(last_plan_before_50):
            dx = px - last_pose_before_50[0]
            dy = py - last_pose_before_50[1]
            dist_sq = dx*dx + dy*dy
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                initial_travel_offset = path_distances[i]
        
        remaining_path_length = max(0.1, total_path_length - initial_travel_offset)
        fake_duration = remaining_path_length / self.config.speed
        
        print(f"[spoof] Initial offset on path: {initial_travel_offset:.2f}m")
        print(f"[spoof] Remaining path: {remaining_path_length:.2f}m")
        print(f"[spoof] Fake movement duration: {fake_duration:.1f}s at {self.config.speed}m/s")
        
        # Generate fake TF/odom and frozen messages from 50% to end
        TFMessage = get_message("tf2_msgs/msg/TFMessage")
        TransformStamped = get_message("geometry_msgs/msg/TransformStamped")
        Odometry = get_message("nav_msgs/msg/Odometry")
        
        # Topics to freeze (not TF/odom) - include tf_static and robot_description
        frozen_topics = [
            f"{ns}/tf_static",
            f"{ns}/robot_description",
            f"{ns}/map",
            f"{ns}/global_costmap/costmap",
            f"{ns}/local_costmap/costmap",
            f"{ns}/sensors/lidar2d_0/scan",
            f"{ns}/plan",
            # "/clock",  # Do NOT freeze /clock - let ros2 bag play generate it!
        ]
        
        # Write tf_static once at the beginning of the fake section
        # This ensures RViz2 gets the static transforms early
        if f"{ns}/tf_static" in last_messages:
            writer.write(f"{ns}/tf_static", last_messages[f"{ns}/tf_static"], t_50_raw_ns + 1000)
            print(f"[spoof] Written tf_static at start of fake section")
        
        # Also write robot_description early
        if f"{ns}/robot_description" in last_messages:
            writer.write(f"{ns}/robot_description", last_messages[f"{ns}/robot_description"], t_50_raw_ns + 1000)
            print(f"[spoof] Written robot_description at start of fake section")
        
        dt = 0.05  # 20 Hz for TF
        t = t_50_raw + dt
        fake_count = 0
        frozen_count = 0
        last_tf_static_time = 0.0  # Track when we last wrote tf_static
        tf_static_interval = 0.5  # Write tf_static every 0.5 seconds
        
        while t < t_end_raw:
            try:
                t_ns = int(t * 1e9)
                elapsed = t - t_50_raw
                
                # Calculate fake position along path
                travel_distance = initial_travel_offset + (elapsed * self.config.speed)
                fake_x, fake_y, fake_yaw = _interpolate_along_path(
                    last_plan_before_50, path_distances, travel_distance
                )
                
                # Calculate simulation time for header
                # Use last_clock_time as base to ensure continuity
                if last_clock_time is not None:
                    sim_t = last_clock_time + elapsed
                else:
                    # Fallback if no clock messages found (unlikely)
                    sim_t = t - self.bag_t0_raw + self.header_t0
                
                if fake_count == 0:
                    print(f"[spoof] Time Debug: t={t:.3f}, bag_t0={self.bag_t0_raw:.3f}, header_t0={self.header_t0:.3f}")
                    print(f"[spoof] Time Debug: last_clock_time={last_clock_time}")
                    print(f"[spoof] Time Debug: Generated sim_t={sim_t:.3f}")

                # Generate fake /clock message FIRST to ensure time is valid for subsequent messages
                clock_msg = get_message("rosgraph_msgs/msg/Clock")()
                clock_msg.clock = _time_to_msg(sim_t)
                writer.write("/clock", serialize_message(clock_msg), t_ns)

                # Generate fake TF message (map->odom and odom->base_link)
                tf_msg = TFMessage()
                
                # map -> odom (identity)
                tf_map_odom = TransformStamped()
                tf_map_odom.header.stamp = _time_to_msg(sim_t)
                tf_map_odom.header.frame_id = "map"
                tf_map_odom.child_frame_id = "odom"
                tf_map_odom.transform.translation.x = 0.0
                tf_map_odom.transform.translation.y = 0.0
                tf_map_odom.transform.translation.z = 0.0
                qx, qy, qz, qw = _quat_from_yaw(0.0)
                tf_map_odom.transform.rotation.x = qx
                tf_map_odom.transform.rotation.y = qy
                tf_map_odom.transform.rotation.z = qz
                tf_map_odom.transform.rotation.w = qw
                tf_msg.transforms.append(tf_map_odom)
                
                # odom -> base_link (fake position)
                tf_odom_base = TransformStamped()
                tf_odom_base.header.stamp = _time_to_msg(sim_t)
                tf_odom_base.header.frame_id = "odom"
                tf_odom_base.child_frame_id = "base_link"
                tf_odom_base.transform.translation.x = fake_x
                tf_odom_base.transform.translation.y = fake_y
                tf_odom_base.transform.translation.z = 0.0
                qx, qy, qz, qw = _quat_from_yaw(fake_yaw)
                tf_odom_base.transform.rotation.x = qx
                tf_odom_base.transform.rotation.y = qy
                tf_odom_base.transform.rotation.z = qz
                tf_odom_base.transform.rotation.w = qw
                tf_msg.transforms.append(tf_odom_base)
                
                # Append ALL accumulated tf_static transforms to every TF message
                if all_tf_static_transforms:
                    for ts in all_tf_static_transforms:
                        # Clone the transform with updated timestamp
                        new_ts = TransformStamped()
                        new_ts.header.stamp = _time_to_msg(sim_t)
                        new_ts.header.frame_id = ts.header.frame_id
                        new_ts.child_frame_id = ts.child_frame_id
                        new_ts.transform.translation.x = ts.transform.translation.x
                        new_ts.transform.translation.y = ts.transform.translation.y
                        new_ts.transform.translation.z = ts.transform.translation.z
                        new_ts.transform.rotation.x = ts.transform.rotation.x
                        new_ts.transform.rotation.y = ts.transform.rotation.y
                        new_ts.transform.rotation.z = ts.transform.rotation.z
                        new_ts.transform.rotation.w = ts.transform.rotation.w
                        tf_msg.transforms.append(new_ts)
                    if fake_count == 0:
                        print(f"[spoof] Added {len(all_tf_static_transforms)} tf_static transforms to each TF message")
                else:
                    if fake_count == 0:
                        print(f"[spoof] WARNING: No tf_static transforms accumulated!")
                
                # Write to namespaced TF topic
                writer.write(f"{ns}/tf", serialize_message(tf_msg), t_ns)
                fake_count += 1
                
                # Generate fake odom message
                odom_msg = Odometry()
                odom_msg.header.stamp = _time_to_msg(sim_t)
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"
                odom_msg.pose.pose.position.x = fake_x
                odom_msg.pose.pose.position.y = fake_y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw
                odom_msg.twist.twist.linear.x = self.config.speed
                
                writer.write(f"{ns}/platform/odom/filtered", serialize_message(odom_msg), t_ns)
                fake_count += 1
                
                # Write frozen topics (less frequently - every 0.1s)
                for topic in frozen_topics:
                    if topic == f"{ns}/tf_static":
                        continue  # Already handled above
                    if topic in last_messages:
                        output_topic = topic_remap.get(topic, topic)
                        writer.write(output_topic, last_messages[topic], t_ns)
                        frozen_count += 1
                        
            except Exception as e:
                import traceback
                print(f"[spoof] ERROR in fake loop: {e}")
                traceback.print_exc()
                break
            
            t += dt
        
        print(f"[spoof] Generated {fake_count} fake TF/odom messages")
        print(f"[spoof] Generated {frozen_count} frozen messages")
        
        # Generate play script and QoS file
        qos_file_path = os.path.join(output_path, "qos_overrides.yaml")
        qos_content = f"""/tf_static:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: transient_local
{ns}/map:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: transient_local
"""
        with open(qos_file_path, 'w') as f:
            f.write(qos_content)
        
        play_script_path = os.path.join(output_path, "play.sh")
        play_script = f"""#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${{BASH_SOURCE[0]}}" )" && pwd )"
ros2 bag play "$SCRIPT_DIR" --qos-profile-overrides-path "$SCRIPT_DIR/qos_overrides.yaml" "$@"
"""
        with open(play_script_path, 'w') as f:
            f.write(play_script)
        os.chmod(play_script_path, 0o755)
        
        print(f"[spoof] Spoofed bag complete: {output_path}")
        print(f"[spoof] To play: cd {output_path} && ./play.sh")
        
        return output_path


def create_spoofed_bag(bag_name: str, output_name: Optional[str] = None) -> str:
    """Create a spoofed bag from the given bag file."""
    bag_path = os.path.join(BAG_DIR, bag_name)
    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Bag file not found: {bag_path}")
    
    spoofer = BagSpoofer(bag_path)
    return spoofer.generate_spoofed_bag(output_name)


def list_spoofed_bags() -> List[Dict[str, any]]:
    """List available spoofed bag files."""
    os.makedirs(SPOOFED_BAG_DIR, exist_ok=True)
    out = []
    for name in sorted(os.listdir(SPOOFED_BAG_DIR)):
        p = os.path.join(SPOOFED_BAG_DIR, name)
        if os.path.isdir(p):
            size = sum(
                os.path.getsize(os.path.join(p, f)) 
                for f in os.listdir(p) 
                if os.path.isfile(os.path.join(p, f))
            )
            out.append({"name": name, "size": size})
    return out
