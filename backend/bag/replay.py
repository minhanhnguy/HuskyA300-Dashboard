# backend/bag/replay.py
"""
Bag file replay and indexing.
"""
from __future__ import annotations

import math
import os
import threading
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String as StringMsg
import rosbag2_py

from ..core import config as C
from .models import StampedTransform, StampedSE2, Scan
from .indexer import (
    BagMapIndex,
    ScanIndex,
    BagCmdIndex,
    BagPlanIndex,
    BagGoalIndex,
    MapMeta,
    se2_compose,
    find_latest_before,
    wrap_pi,
)
from .reader import BAG_DIR, SPOOFED_BAG_DIR, topic_map, norm_ns


def _yaw_from_quat(q) -> float:
    """Extract yaw angle from quaternion."""
    ysqr = q.y * q.y
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
    return float(np.arctan2(t3, t4))


def replay_bag_in_thread(shared, name: str, speed: float = 1.0, spoofed: bool = False):
    """
    Index bag (once) and populate:
      - core.pose_history ([(t,x,y,yaw)...]) in MAP frame if TF is available
      - core.path
      - core._bag_map_index (for /api/v1/map_full_at)
      - core.map_info, core.map_grid, core.map_version (final snapshot)
      - core._bag_scan_index, core._bag_tf_pairs, core._bag_names (for /api/v1/scan_at)
      - core._bag_cmd_index (for /api/v1/cmd_stats_at)
      - core.bag_cmd_* prefix arrays (legacy/compatibility)

    If spoofed=True, loads from spoofed_bags/ directory instead of bag_files/.
    """
    if spoofed:
        path = os.path.join(SPOOFED_BAG_DIR, name)
    else:
        path = os.path.join(BAG_DIR, name)
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    # CRITICAL: Clear old state BEFORE starting indexing thread
    core = getattr(shared, "core", shared)
    with core.lock:
        print(f"[bag] Clearing old state before loading: {name}")
        core.pose_history = deque(maxlen=getattr(C, "POSE_HISTORY_MAX", 100000))
        if hasattr(core, "path") and core.path is not None:
            core.path.clear()
        core._bag_map_index = None
        core._bag_global_costmap_index = None
        core._bag_local_costmap_index = None
        core._bag_scan_index = None
        core._bag_cmd_index = None
        core._bag_plan_index = None
        core._bag_goal_index = None
        core._bag_tf_pairs = {}
        core._bag_tf_3d_pairs = {}
        core._bag_tf_static_list = []

    def _worker():
        try:
            print(f"[bag] Opening {path}")
            storage = rosbag2_py.StorageOptions(uri=path, storage_id="mcap")
            converter = rosbag2_py.ConverterOptions("", "")
            reader = rosbag2_py.SequentialReader()
            reader.open(storage, converter)

            topics = topic_map(reader)
            print(f"[bag] Topics in bag:")
            for k, v in topics.items():
                print(f"   - {k}: {v}")

            ns = norm_ns(getattr(C, "NAMESPACE", "/a300_0000"))

            # Exact topic preferences
            odom_exact = f"{ns}/platform/odom/filtered"
            map_exact = f"{ns}/map"
            mapup_exact = f"{ns}/map_updates"
            tf_exact = f"{ns}/tf"
            tfstat_exact = f"{ns}/tf_static"
            scan_exact = f"{ns}/sensors/lidar2d_0/scan"
            gc_exact = f"{ns}/global_costmap/costmap"
            lc_exact = f"{ns}/local_costmap/costmap"
            plan_exact = f"{ns}/plan"
            goal_exact = f"{ns}/goal_pose"
            robot_desc_topic = f"{ns}/robot_description"

            robot_description: Optional[str] = None

            # Msg types
            Odom = get_message("nav_msgs/msg/Odometry")
            OccGrid = get_message("nav_msgs/msg/OccupancyGrid")
            OccUp = get_message("map_msgs/msg/OccupancyGridUpdate")
            TFMsg = get_message("tf2_msgs/msg/TFMessage")
            PoseSt = get_message("geometry_msgs/msg/PoseStamped")
            Laser = get_message("sensor_msgs/msg/LaserScan")
            Twist = get_message("geometry_msgs/msg/Twist")
            TwistStamped = get_message("geometry_msgs/msg/TwistStamped")
            PathMsg = get_message("nav_msgs/msg/Path")

            # Resolve topics
            def pick_exact_or_any(exact_name: str, type_suffix: str) -> Optional[str]:
                if exact_name in topics and topics[exact_name].endswith(type_suffix):
                    return exact_name
                for tname, ttype in topics.items():
                    if ttype.endswith(type_suffix):
                        return tname
                return None

            odom_topic = (
                odom_exact
                if (
                    odom_exact in topics
                    and topics[odom_exact].endswith("nav_msgs/msg/Odometry")
                )
                else None
            )
            if odom_topic is None:
                for tname, ttype in topics.items():
                    if ttype.endswith("nav_msgs/msg/Odometry"):
                        odom_topic = tname
                        break

            map_topic = pick_exact_or_any(map_exact, "nav_msgs/msg/OccupancyGrid")
            map_up_topic = pick_exact_or_any(mapup_exact, "map_msgs/msg/OccupancyGridUpdate")

            # TF topics
            tf_topics: List[str] = []
            for cand in [tf_exact, tfstat_exact, "/tf", "/tf_static"]:
                if cand in topics and topics[cand].endswith("tf2_msgs/msg/TFMessage"):
                    tf_topics.append(cand)
            for tname, ttype in topics.items():
                if ttype.endswith("tf2_msgs/msg/TFMessage") and tname not in tf_topics:
                    tf_topics.append(tname)

            # Pose topic (fallback if no odom)
            pose_topic: Optional[str] = None
            pose_exact = f"{ns}/pose"
            if (
                odom_topic is None
                and pose_exact in topics
                and topics[pose_exact].endswith("geometry_msgs/msg/PoseStamped")
            ):
                pose_topic = pose_exact

            # Scan topic
            scan_topic: Optional[str] = None
            if (
                scan_exact in topics
                and topics[scan_exact].endswith("sensor_msgs/msg/LaserScan")
            ):
                scan_topic = scan_exact
            else:
                for tname, ttype in topics.items():
                    if ttype.endswith("sensor_msgs/msg/LaserScan"):
                        scan_topic = tname
                        break

            # Costmap topics
            gc_topic = pick_exact_or_any(gc_exact, "nav_msgs/msg/OccupancyGrid")
            lc_topic = pick_exact_or_any(lc_exact, "nav_msgs/msg/OccupancyGrid")

            # Plan/Goal topics
            plan_topic = pick_exact_or_any(plan_exact, "nav_msgs/msg/Path")
            goal_topic = pick_exact_or_any(goal_exact, "geometry_msgs/msg/PoseStamped")

            # Cmd_vel topics
            cmd_topics: List[str] = []
            for tname, ttype in topics.items():
                if (
                    ttype.endswith("geometry_msgs/msg/Twist")
                    or ttype.endswith("geometry_msgs/msg/TwistStamped")
                ):
                    cmd_topics.append(tname)

            preferred_cmds: List[str] = []
            for tname in cmd_topics:
                if (
                    tname == f"{ns}/cmd_vel"
                    or tname == "/cmd_vel"
                    or tname.endswith("/cmd_vel")
                ):
                    preferred_cmds.append(tname)
            if preferred_cmds:
                cmd_topics = preferred_cmds

            print(f"[bag] Selected topics:")
            print(f"   odom: {odom_topic}")
            print(f"   pose: {pose_topic}")
            print(f"   map:  {map_topic}")
            print(f"   up:   {map_up_topic}")
            print(f"   gc:   {gc_topic}")
            print(f"   lc:   {lc_topic}")
            print(f"   tf*:  {tf_topics}")
            print(f"   scan: {scan_topic}")
            print(f"   cmd:  {cmd_topics}")
            print(f"   plan: {plan_topic}")
            print(f"   goal: {goal_topic}")

            # Storage
            poses_odom: List[Tuple[float, float, float, float]] = []
            path_pts: List[Tuple[float, float]] = []
            MOVE_EPS = getattr(C, "MOVE_EPS", 0.02)

            map_index = BagMapIndex()
            gc_index = BagMapIndex()
            lc_index = BagMapIndex()
            first_map_meta: Optional[MapMeta] = None

            # TF timeline candidates
            tf_map_odom: List[StampedSE2] = []
            tf_odom_base: List[StampedSE2] = []

            # Generic TF pairs for scan service
            tf_pairs: Dict[Tuple[str, str], List[StampedSE2]] = {}

            # Full 3D TF pairs for RViz visualization
            tf_3d_pairs: Dict[Tuple[str, str], List[StampedTransform]] = {}
            tf_static_list: List[TransformStamped] = []

            # Frame candidates
            map_candidates = ["map", f"{ns}/map", "/map"]
            odom_candidates = ["odom", f"{ns}/odom", "/odom"]
            base_candidates = [
                "base_link",
                f"{ns}/base_link",
                "base_footprint",
                f"{ns}/base_footprint",
                "/base_link",
                "/base_footprint",
            ]

            # Indices
            scan_index = ScanIndex()
            cmd_index = BagCmdIndex()
            plan_index = BagPlanIndex()
            goal_index = BagGoalIndex()

            # Iterate bag
            while reader.has_next():
                topic, raw, _t_ns = reader.read_next()

                if odom_topic and topic == odom_topic:
                    msg = deserialize_message(raw, Odom)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    px = float(msg.pose.pose.position.x)
                    py = float(msg.pose.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.pose.orientation)
                    poses_odom.append((t, px, py, yaw))
                    if (
                        not path_pts
                        or (px - path_pts[-1][0]) ** 2 + (py - path_pts[-1][1]) ** 2 >= MOVE_EPS**2
                    ):
                        path_pts.append((px, py))

                elif pose_topic and topic == pose_topic:
                    msg = deserialize_message(raw, PoseSt)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    px = float(msg.pose.position.x)
                    py = float(msg.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.orientation)
                    poses_odom.append((t, px, py, yaw))
                    if (
                        not path_pts
                        or (px - path_pts[-1][0]) ** 2 + (py - path_pts[-1][1]) ** 2 >= MOVE_EPS**2
                    ):
                        path_pts.append((px, py))

                elif map_topic and topic == map_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    map_index.add_full(
                        t=t, w=w, h=h, res=float(info.resolution),
                        ox=float(info.origin.position.x), oy=float(info.origin.position.y),
                        oyaw=float(oyaw), frame_id=msg.header.frame_id, data=arr,
                    )
                    if first_map_meta is None:
                        first_map_meta = MapMeta(
                            w, h, float(info.resolution),
                            float(info.origin.position.x), float(info.origin.position.y),
                            float(oyaw), version=1, frame_id=msg.header.frame_id,
                        )

                elif map_up_topic and topic == map_up_topic:
                    msg = deserialize_message(raw, OccUp)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    x0, y0 = int(msg.x), int(msg.y)
                    w, h = int(msg.width), int(msg.height)
                    flat = np.asarray(msg.data, dtype=np.int8)
                    map_index.add_patch(t=t, x=x0, y=y0, w=w, h=h, flat=flat)

                elif gc_topic and topic == gc_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    gc_index.add_full(
                        t=t, w=w, h=h, res=float(info.resolution),
                        ox=float(info.origin.position.x), oy=float(info.origin.position.y),
                        oyaw=float(oyaw), frame_id=msg.header.frame_id, data=arr,
                    )

                elif lc_topic and topic == lc_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    lc_index.add_full(
                        t=t, w=w, h=h, res=float(info.resolution),
                        ox=float(info.origin.position.x), oy=float(info.origin.position.y),
                        oyaw=float(oyaw), frame_id=msg.header.frame_id, data=arr,
                    )

                elif topic in tf_topics:
                    msg = deserialize_message(raw, TFMsg)
                    is_static = "static" in topic

                    for ts in msg.transforms:
                        if is_static:
                            tf_static_list.append(ts)
                            continue

                        t = float(ts.header.stamp.sec) + float(ts.header.stamp.nanosec) * 1e-9
                        parent = ts.header.frame_id.strip()
                        child = ts.child_frame_id.strip()

                        x = float(ts.transform.translation.x)
                        y = float(ts.transform.translation.y)
                        yaw = _yaw_from_quat(ts.transform.rotation)
                        se = StampedSE2(t=t, x=x, y=y, yaw=yaw)
                        tf_pairs.setdefault((parent, child), []).append(se)

                        if (parent in map_candidates) and (child in odom_candidates):
                            tf_map_odom.append(se)
                        if (parent in odom_candidates) and (child in base_candidates):
                            tf_odom_base.append(se)

                        st = StampedTransform(
                            t=t,
                            tx=float(ts.transform.translation.x),
                            ty=float(ts.transform.translation.y),
                            tz=float(ts.transform.translation.z),
                            qx=float(ts.transform.rotation.x),
                            qy=float(ts.transform.rotation.y),
                            qz=float(ts.transform.rotation.z),
                            qw=float(ts.transform.rotation.w),
                        )
                        tf_3d_pairs.setdefault((parent, child), []).append(st)

                elif scan_topic and topic == scan_topic:
                    msg = deserialize_message(raw, Laser)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    frame = (msg.header.frame_id or "").strip()
                    scan = Scan(
                        t=t, frame=frame,
                        angle_min=float(msg.angle_min), angle_inc=float(msg.angle_increment),
                        range_min=float(msg.range_min), range_max=float(msg.range_max),
                        ranges=[float(r) for r in msg.ranges],
                    )
                    scan_index.add(scan)

                elif topic in cmd_topics:
                    ttype = topics.get(topic, "")
                    if ttype.endswith("geometry_msgs/msg/Twist"):
                        msg = deserialize_message(raw, Twist)
                        t = float(_t_ns) * 1e-9
                        cmd_index.add(
                            t,
                            float(getattr(msg.linear, "x", 0.0)),
                            float(getattr(msg.linear, "y", 0.0)),
                            float(getattr(msg.linear, "z", 0.0)),
                            float(getattr(msg.angular, "x", 0.0)),
                            float(getattr(msg.angular, "y", 0.0)),
                            float(getattr(msg.angular, "z", 0.0)),
                        )
                    elif ttype.endswith("geometry_msgs/msg/TwistStamped"):
                        try:
                            msg = deserialize_message(raw, TwistStamped)
                            t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                            twist = msg.twist
                            cmd_index.add(
                                t,
                                float(getattr(twist.linear, "x", 0.0)),
                                float(getattr(twist.linear, "y", 0.0)),
                                float(getattr(twist.linear, "z", 0.0)),
                                float(getattr(twist.angular, "x", 0.0)),
                                float(getattr(twist.angular, "y", 0.0)),
                                float(getattr(twist.angular, "z", 0.0)),
                            )
                        except Exception:
                            pass

                elif plan_topic and topic == plan_topic:
                    msg = deserialize_message(raw, PathMsg)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    poses = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
                    plan_index.add(t, poses)

                elif goal_topic and topic == goal_topic:
                    msg = deserialize_message(raw, PoseSt)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    px = float(msg.pose.position.x)
                    py = float(msg.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.orientation)
                    goal_index.add(t, px, py, yaw)

                elif robot_desc_topic and topic == robot_desc_topic:
                    if robot_description is None:
                        msg = deserialize_message(raw, StringMsg)
                        robot_description = msg.data
                        print(f"[bag] Found robot_description in {topic} (len={len(robot_description)})")

            # Sort and finalize
            poses_odom.sort(key=lambda p: p[0])
            tf_map_odom.sort(key=lambda s: s.t)
            tf_odom_base.sort(key=lambda s: s.t)
            for k in tf_3d_pairs:
                tf_3d_pairs[k].sort(key=lambda s: s.t)
            scan_index.finalize()
            cmd_index.finalize()
            plan_index.finalize()
            goal_index.finalize()

            print(f"[bag] TF capture stats:")
            print(f"   poses_odom (odometry): {len(poses_odom)}")
            print(f"   tf_map_odom (map→odom): {len(tf_map_odom)}")
            print(f"   tf_odom_base (odom→base): {len(tf_odom_base)}")
            print(f"   BAG_USE_TF config: {getattr(C, 'BAG_USE_TF', True)}")

            # Compute poses using TF
            poses_final: List[Tuple[float, float, float, float]] = []
            path_pts = []

            if getattr(C, "BAG_USE_TF", True) and tf_odom_base:
                if tf_map_odom:
                    print(f"[bag] Using TF chain: map→odom→base (composing {len(tf_odom_base)} transforms)")
                    poses_map: List[Tuple[float, float, float, float]] = []
                    i_tf = 0
                    for ob in tf_odom_base:
                        mo, i_tf = find_latest_before(tf_map_odom, ob.t, i_tf)
                        map_of_odom = StampedSE2(t=mo.t, x=mo.x, y=mo.y, yaw=mo.yaw)
                        mb = se2_compose(map_of_odom, ob)
                        poses_map.append((ob.t, mb.x, mb.y, mb.yaw))
                    poses_final = poses_map
                else:
                    print(f"[bag] Using TF odom→base only ({len(tf_odom_base)} transforms, no map→odom)")
                    poses_final = [(ob.t, ob.x, ob.y, ob.yaw) for ob in tf_odom_base]

                for (_, x, y, _) in poses_final:
                    if not path_pts or (x - path_pts[-1][0])**2 + (y - path_pts[-1][1])**2 >= MOVE_EPS**2:
                        path_pts.append((x, y))
            else:
                print(f"[bag] FALLBACK: No TF odom→base data, using odometry ({len(poses_odom)} poses)")
                if getattr(C, "BAG_USE_TF", True) and poses_odom and tf_map_odom:
                    print(f"[bag]   Composing odometry with map→odom ({len(tf_map_odom)} transforms)")
                    poses_map = []
                    i_tf = 0
                    for (t, xo, yo, yaw_ob) in poses_odom:
                        mo, i_tf = find_latest_before(tf_map_odom, t, i_tf)
                        map_of_odom = StampedSE2(t=mo.t, x=mo.x, y=mo.y, yaw=mo.yaw)
                        odom_of_base = StampedSE2(t=t, x=xo, y=yo, yaw=yaw_ob)
                        mb = se2_compose(map_of_odom, odom_of_base)
                        poses_map.append((t, mb.x, mb.y, mb.yaw))
                    for (_, x, y, _) in poses_map:
                        if not path_pts or (x - path_pts[-1][0])**2 + (y - path_pts[-1][1])**2 >= MOVE_EPS**2:
                            path_pts.append((x, y))
                    poses_final = poses_map
                else:
                    poses_final = poses_odom
                    if getattr(C, "BAG_ANCHOR_ODOM_TO_MAP", True) and poses_odom and map_index._versions:
                        if first_map_meta is None:
                            first_map_meta = map_index._versions[0][1]
                        mx0 = float(first_map_meta.origin_x)
                        my0 = float(first_map_meta.origin_y)
                        myaw0 = float(first_map_meta.origin_yaw) if getattr(C, "BAG_USE_MAP_YAW", True) else 0.0

                        t0, x0, y0, yaw0 = poses_odom[0]
                        dtheta = myaw0 - yaw0
                        c = math.cos(dtheta)
                        s = math.sin(dtheta)

                        anchored: List[Tuple[float, float, float, float]] = []
                        for (t, x, y, yaw) in poses_odom:
                            dx, dy = (x - x0), (y - y0)
                            xr = dx * c - dy * s
                            yr = dx * s + dy * c
                            xm = mx0 + xr
                            ym = my0 + yr
                            yawm = wrap_pi((yaw - yaw0) + myaw0)
                            anchored.append((t, xm, ym, yawm))
                        poses_final = anchored

                    path_pts = []
                    for (_, x, y, _) in poses_final:
                        if not path_pts or (x - path_pts[-1][0])**2 + (y - path_pts[-1][1])**2 >= MOVE_EPS**2:
                            path_pts.append((x, y))

            # Build cmd_vel prefix arrays
            if cmd_index.times:
                bag_cmd_times = list(cmd_index.times)
                bag_cmd_vx = list(cmd_index.vx)
                bag_cmd_vy = list(cmd_index.vy)
                bag_cmd_vz = list(cmd_index.vz)
                bag_cmd_wx = list(cmd_index.wx)
                bag_cmd_wy = list(cmd_index.wy)
                bag_cmd_wz = list(cmd_index.wz)
                bag_cmd_prefix_max_vx_fwd = list(cmd_index.max_vx_forward_prefix)
                bag_cmd_prefix_max_vx_rev = list(cmd_index.max_vx_reverse_prefix)
                bag_cmd_prefix_max_wz_abs = list(cmd_index.max_wz_abs_prefix)
                bag_cmd_prefix_has_lateral = list(cmd_index.has_lateral_prefix)
                bag_cmd_prefix_has_ang_xy = list(cmd_index.has_ang_xy_prefix)
                bag_cmd_prefix_avg_vx = list(cmd_index.avg_vx_prefix)
                bag_cmd_prefix_max_vy_abs = list(cmd_index.max_vy_abs_prefix)
                bag_cmd_prefix_avg_vy_abs = list(cmd_index.avg_vy_abs_prefix)
                bag_cmd_t0 = cmd_index.bag_t0 if cmd_index.bag_t0 is not None else cmd_index.times[0]
                bag_cmd_t1 = cmd_index.bag_t1 if cmd_index.bag_t1 is not None else cmd_index.times[-1]
            else:
                bag_cmd_times = bag_cmd_vx = bag_cmd_vy = bag_cmd_vz = []
                bag_cmd_wx = bag_cmd_wy = bag_cmd_wz = []
                bag_cmd_prefix_max_vx_fwd = bag_cmd_prefix_max_vx_rev = []
                bag_cmd_prefix_max_wz_abs = []
                bag_cmd_prefix_has_lateral = bag_cmd_prefix_has_ang_xy = []
                bag_cmd_prefix_avg_vx = bag_cmd_prefix_max_vy_abs = bag_cmd_prefix_avg_vy_abs = []
                bag_cmd_t0 = bag_cmd_t1 = 0.0

            # Commit to shared / core
            core = getattr(shared, "core", shared)
            with core.lock:
                max_ph = getattr(C, "POSE_HISTORY_MAX", 100000)
                core.pose_history = deque(poses_final, maxlen=max_ph)

                if not hasattr(core, "path") or core.path is None:
                    max_pts = getattr(C, "PATH_MAX_POINTS", 10000)
                    core.path = deque(maxlen=max_pts)
                else:
                    try:
                        core.path.clear()
                    except Exception:
                        max_pts = getattr(C, "PATH_MAX_POINTS", 10000)
                        core.path = deque(maxlen=max_pts)
                for pt in path_pts:
                    core.path.append(pt)

                end_t = poses_final[-1][0] if poses_final else 0.0
                snap = map_index.snapshot_at(end_t)
                if snap:
                    meta, grid = snap
                    core.map_grid = grid
                    mv = getattr(core, "map_version", 0)
                    core.map_version = int(mv) + 1
                    core.map_info = {
                        "width": int(meta.width),
                        "height": int(meta.height),
                        "resolution": float(meta.resolution),
                        "origin": {
                            "x": float(meta.origin_x),
                            "y": float(meta.origin_y),
                            "yaw": float(meta.origin_yaw),
                        },
                        "version": int(core.map_version),
                        "tile_size": int(getattr(C, "MAP_TILE_SIZE", 256)),
                    }

                core._bag_map_index = map_index
                core._bag_global_costmap_index = gc_index
                core._bag_local_costmap_index = lc_index
                core._bag_scan_index = scan_index if scan_index._scans else None
                core._bag_cmd_index = cmd_index if cmd_index.times else None
                core._bag_plan_index = plan_index
                core._bag_goal_index = goal_index
                core.robot_description = robot_description

                core._bag_tf_pairs = tf_pairs
                core._bag_tf_3d_pairs = tf_3d_pairs
                core._bag_tf_static_list = tf_static_list
                core._bag_names = {
                    "ns": ns,
                    "map_candidates": map_candidates,
                    "odom_candidates": odom_candidates,
                    "base_candidates": base_candidates,
                }

                core.bag_cmd_times = bag_cmd_times
                core.bag_cmd_vx = bag_cmd_vx
                core.bag_cmd_vy = bag_cmd_vy
                core.bag_cmd_vz = bag_cmd_vz
                core.bag_cmd_wx = bag_cmd_wx
                core.bag_cmd_wy = bag_cmd_wy
                core.bag_cmd_wz = bag_cmd_wz
                core.bag_cmd_prefix_max_vx_fwd = bag_cmd_prefix_max_vx_fwd
                core.bag_cmd_prefix_max_vx_rev = bag_cmd_prefix_max_vx_rev
                core.bag_cmd_prefix_max_wz_abs = bag_cmd_prefix_max_wz_abs
                core.bag_cmd_prefix_has_lateral = bag_cmd_prefix_has_lateral
                core.bag_cmd_prefix_has_ang_xy = bag_cmd_prefix_has_ang_xy
                core.bag_cmd_prefix_avg_vx = bag_cmd_prefix_avg_vx
                core.bag_cmd_prefix_max_vy_abs = bag_cmd_prefix_max_vy_abs
                core.bag_cmd_prefix_avg_vy_abs = bag_cmd_prefix_avg_vy_abs
                core.bag_cmd_t0 = bag_cmd_t0
                core.bag_cmd_t1 = bag_cmd_t1

            print(
                f"[bag] Index complete: poses={len(poses_final)}, path_pts={len(path_pts)}, "
                f"map_versions={len(map_index._versions)}, tf_map_odom={len(tf_map_odom)}, "
                f"tf_odom_base={len(tf_odom_base)}, scans={len(scan_index._scans)}, "
                f"cmd_samples={len(cmd_index.times)}"
            )

        except Exception as e:
            print(f"[bag] Error: {e}")
            import traceback
            traceback.print_exc()

    threading.Thread(target=_worker, daemon=True).start()
