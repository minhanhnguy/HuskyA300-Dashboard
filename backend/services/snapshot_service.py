
import math
import numpy as np
from typing import Any, Dict, List, Optional, Tuple

from geometry_msgs.msg import TransformStamped, Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time as RosTime
from std_msgs.msg import Header

from ..bag_replay import _StampedSE2, _find_latest_before

def _get_interpolated_se2(stamps: List[_StampedSE2], t: float) -> Optional[_StampedSE2]:
    if not stamps:
        return None
    # stamps is sorted by t
    # find latest before t
    # We can use _find_latest_before from bag_replay but we need to import it or reimplement
    # Reimplementing binary search for simplicity and independence
    
    if t <= stamps[0].t:
        return stamps[0]
    if t >= stamps[-1].t:
        return stamps[-1]
        
    # Binary search
    lo, hi = 0, len(stamps) - 1
    idx = 0
    while lo <= hi:
        mid = (lo + hi) // 2
        if stamps[mid].t <= t:
            idx = mid
            lo = mid + 1
        else:
            hi = mid - 1
            
    # stamps[idx] is <= t
    # stamps[idx+1] is > t
    a = stamps[idx]
    if idx + 1 >= len(stamps):
        return a
    b = stamps[idx+1]
    
    dt = b.t - a.t
    if dt < 1e-6:
        return a
        
    u = (t - a.t) / dt
    x = a.x + (b.x - a.x) * u
    y = a.y + (b.y - a.y) * u
    
    # Yaw interpolation
    da = b.yaw - a.yaw
    while da > math.pi: da -= 2*math.pi
    while da <= -math.pi: da += 2*math.pi
    yaw = a.yaw + da * u
    
    return _StampedSE2(t, x, y, yaw)

def _se2_to_transform_stamped(se2: _StampedSE2, parent: str, child: str, t: float) -> TransformStamped:
    msg = TransformStamped()
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = se2.x
    msg.transform.translation.y = se2.y
    msg.transform.translation.z = 0.0
    
    cy = math.cos(se2.yaw * 0.5)
    sy = math.sin(se2.yaw * 0.5)
    msg.transform.rotation.w = cy
    msg.transform.rotation.z = sy
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    return msg

def get_bag_snapshot_at_service(core, t: float) -> Dict[str, Any]:
    snapshot = {}
    
    # 1. TF
    # We need to publish ALL available transforms to reconstruct the robot tree.
    # core has tf_3d_pairs (dict of list of _StampedTransform) and tf_static_list
    tf_msg = TFMessage()
    
    with core.lock:
        tf_3d_pairs = getattr(core, "_bag_tf_3d_pairs", {})
        tf_static_list = getattr(core, "_bag_tf_static_list", [])
        
        # Add static TFs (if any)
        # We can publish them in /tf_static or /tf depending on how we want to handle it.
        # Ideally, we should return a separate tf_static_msg if we want to publish to /tf_static.
        # But for now, let's put everything in tf_msg (which goes to /tf) OR handle tf_static separately.
        # The caller (ros_worker) publishes to /tf and /tf_static.
        # Let's return "tf_static_msg" in snapshot if we have static TFs.
        if tf_static_list:
             snapshot["tf_static_msg"] = TFMessage(transforms=tf_static_list)

        # Interpolate dynamic TFs
        for (parent, child), stamps in tf_3d_pairs.items():
            if not stamps:
                continue
                
            # Binary search for t
            # stamps is list of _StampedTransform, sorted by t
            # ... (reimplement binary search or helper)
            
            # Helper for 3D interpolation
            def get_interp_3d(stamps, t):
                if t <= stamps[0].t: return stamps[0]
                if t >= stamps[-1].t: return stamps[-1]
                
                lo, hi = 0, len(stamps) - 1
                idx = 0
                while lo <= hi:
                    mid = (lo + hi) // 2
                    if stamps[mid].t <= t:
                        idx = mid
                        lo = mid + 1
                    else:
                        hi = mid - 1
                
                a = stamps[idx]
                if idx + 1 >= len(stamps): return a
                b = stamps[idx+1]
                
                dt = b.t - a.t
                if dt < 1e-6: return a
                
                u = (t - a.t) / dt
                
                # LERP translation
                tx = a.tx + (b.tx - a.tx) * u
                ty = a.ty + (b.ty - a.ty) * u
                tz = a.tz + (b.tz - a.tz) * u
                
                # SLERP rotation (simplified)
                # dot = q1.q2
                dot = a.qx*b.qx + a.qy*b.qy + a.qz*b.qz + a.qw*b.qw
                
                # If dot < 0, invert one quaternion to take shortest path
                b_qx, b_qy, b_qz, b_qw = b.qx, b.qy, b.qz, b.qw
                if dot < 0:
                    b_qx, b_qy, b_qz, b_qw = -b_qx, -b_qy, -b_qz, -b_qw
                    dot = -dot
                
                if dot > 0.9995:
                    # Linear interpolation for small angles
                    qx = a.qx + (b_qx - a.qx) * u
                    qy = a.qy + (b_qy - a.qy) * u
                    qz = a.qz + (b_qz - a.qz) * u
                    qw = a.qw + (b_qw - a.qw) * u
                    # Normalize
                    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
                    return (tx, ty, tz, qx/norm, qy/norm, qz/norm, qw/norm)
                
                theta_0 = math.acos(dot)
                theta = theta_0 * u
                sin_theta = math.sin(theta)
                sin_theta_0 = math.sin(theta_0)
                
                s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
                s1 = sin_theta / sin_theta_0
                
                qx = s0 * a.qx + s1 * b_qx
                qy = s0 * a.qy + s1 * b_qy
                qz = s0 * a.qz + s1 * b_qz
                qw = s0 * a.qw + s1 * b_qw
                
                return (tx, ty, tz, qx, qy, qz, qw)

            res = get_interp_3d(stamps, t)
            if res:
                # res is either _StampedTransform or tuple
                if isinstance(res, tuple):
                    tx, ty, tz, qx, qy, qz, qw = res
                else:
                    tx, ty, tz, qx, qy, qz, qw = res.tx, res.ty, res.tz, res.qx, res.qy, res.qz, res.qw
                    
                ts = TransformStamped()
                sec = int(t)
                nanosec = int((t - sec) * 1e9)
                ts.header.stamp = RosTime(sec=sec, nanosec=nanosec)
                ts.header.frame_id = parent
                ts.child_frame_id = child
                ts.transform.translation.x = tx
                ts.transform.translation.y = ty
                ts.transform.translation.z = tz
                ts.transform.rotation.x = qx
                ts.transform.rotation.y = qy
                ts.transform.rotation.z = qz
                ts.transform.rotation.w = qw
                tf_msg.transforms.append(ts)

    if tf_msg.transforms:
        snapshot["tf_msg"] = tf_msg
        
    # 2. Map
    with core.lock:
        idx = getattr(core, "_bag_map_index", None)
    if idx:
        res = idx.snapshot_at(t)
        if res:
            meta, grid = res
            map_msg = OccupancyGrid()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            map_msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
            map_msg.header.frame_id = meta.frame_id
            map_msg.info.resolution = meta.resolution
            map_msg.info.width = meta.width
            map_msg.info.height = meta.height
            map_msg.info.origin.position.x = meta.origin_x
            map_msg.info.origin.position.y = meta.origin_y
            map_msg.info.origin.position.z = 0.0
            
            cy = math.cos(meta.origin_yaw * 0.5)
            sy = math.sin(meta.origin_yaw * 0.5)
            map_msg.info.origin.orientation.w = cy
            map_msg.info.origin.orientation.z = sy
            
            # grid is numpy int8, need to flatten
            map_msg.data = grid.flatten(order="C").tolist()
            snapshot["map"] = map_msg

    # 2b. Global Costmap
    with core.lock:
        idx = getattr(core, "_bag_global_costmap_index", None)
    if idx:
        res = idx.snapshot_at(t)
        if res:
            meta, grid = res
            gc_msg = OccupancyGrid()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            gc_msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
            gc_msg.header.frame_id = meta.frame_id
            gc_msg.info.resolution = meta.resolution
            gc_msg.info.width = meta.width
            gc_msg.info.height = meta.height
            gc_msg.info.origin.position.x = meta.origin_x
            gc_msg.info.origin.position.y = meta.origin_y
            gc_msg.info.origin.position.z = 0.0
            
            cy = math.cos(meta.origin_yaw * 0.5)
            sy = math.sin(meta.origin_yaw * 0.5)
            gc_msg.info.origin.orientation.w = cy
            gc_msg.info.origin.orientation.z = sy
            
            gc_msg.data = grid.flatten(order="C").tolist()
            snapshot["gc"] = gc_msg

    # 2c. Local Costmap
    with core.lock:
        idx = getattr(core, "_bag_local_costmap_index", None)
    if idx:
        res = idx.snapshot_at(t)
        if res:
            meta, grid = res
            lc_msg = OccupancyGrid()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            lc_msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
            lc_msg.header.frame_id = meta.frame_id
            lc_msg.info.resolution = meta.resolution
            lc_msg.info.width = meta.width
            lc_msg.info.height = meta.height
            lc_msg.info.origin.position.x = meta.origin_x
            lc_msg.info.origin.position.y = meta.origin_y
            lc_msg.info.origin.position.z = 0.0
            
            cy = math.cos(meta.origin_yaw * 0.5)
            sy = math.sin(meta.origin_yaw * 0.5)
            lc_msg.info.origin.orientation.w = cy
            lc_msg.info.origin.orientation.z = sy
            
            lc_msg.data = grid.flatten(order="C").tolist()
            snapshot["lc"] = lc_msg

    # 3. Scan
    with core.lock:
        idx = getattr(core, "_bag_scan_index", None)
    if idx:
        scan = idx.nearest(t)
        if scan:
            scan_msg = LaserScan()
            sec = int(scan.t)
            nanosec = int((scan.t - sec) * 1e9)
            scan_msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
            scan_msg.header.frame_id = scan.frame
            scan_msg.angle_min = scan.angle_min
            scan_msg.angle_max = scan.angle_min + scan.angle_inc * (len(scan.ranges) - 1)
            scan_msg.angle_increment = scan.angle_inc
            scan_msg.range_min = scan.range_min
            scan_msg.range_max = scan.range_max
            scan_msg.ranges = scan.ranges
            snapshot["scan"] = scan_msg

    # 4. Plan
    with core.lock:
        idx = getattr(core, "_bag_plan_index", None)
    if idx:
        poses = idx.nearest(t)
        if poses:
            path_msg = Path()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            path_msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
            path_msg.header.frame_id = "map" # Assuming plan is in map frame
            
            for (px, py) in poses:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = px
                ps.pose.position.y = py
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            snapshot["plan"] = path_msg

    # 5. Goal
    with core.lock:
        idx = getattr(core, "_bag_goal_index", None)
    if idx:
        goal = idx.latest_at(t)
        if goal:
            x, y, yaw = goal
            goal_msg = PoseStamped()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            goal_msg.header.stamp = RosTime(sec=sec, nanosec=nanosec)
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            goal_msg.pose.orientation.w = cy
            goal_msg.pose.orientation.z = sy
            snapshot["goal"] = goal_msg

    # 6. Robot Description
    with core.lock:
        desc = getattr(core, "robot_description", None)
    if desc:
        snapshot["robot_description"] = desc

    return snapshot
