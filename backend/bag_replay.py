# backend/bag_replay.py
import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import math
from collections import deque

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from . import config as C

BAG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "bag_files"))

# ----------------- utils -----------------


def _yaw_from_quat(q):
    ysqr = q.y * q.y
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)
    return float(np.arctan2(t3, t4))


def _wrap_pi(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def _topic_map(reader) -> Dict[str, str]:
    out = {}
    for md in reader.get_all_topics_and_types():
        out[md.name] = md.type
    return out


def _norm_ns(ns: str) -> str:
    if not ns:
        return ""
    return ns if ns.startswith("/") else ("/" + ns)


# ----------------- time-travel map index -----------------


@dataclass
class _MapMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    version: int


class _BagMapIndex:
    """
    Keeps full-map versions + patches; reconstructs exact map state at time t.
    """

    def __init__(self):
        self._versions: List[Tuple[float, _MapMeta, np.ndarray]] = []
        self._patches: Dict[int, List[Tuple[float, Tuple[int, int, int, int, np.ndarray]]]] = {}
        self._cache_version: Optional[int] = None
        self._cache_t: float = -1.0
        self._cache_grid: Optional[np.ndarray] = None

    def add_full(
        self,
        t: float,
        w: int,
        h: int,
        res: float,
        ox: float,
        oy: float,
        oyaw: float,
        data: np.ndarray,
    ):
        version = len(self._versions) + 1
        meta = _MapMeta(w, h, res, ox, oy, oyaw, version)
        base = np.asarray(data, dtype=np.int8).reshape(h, w).copy(order="C")
        self._versions.append((t, meta, base))
        self._patches[version] = []
        self._cache_version = None
        self._cache_t = -1.0
        self._cache_grid = None

    def add_patch(self, t: float, x: int, y: int, w: int, h: int, flat: np.ndarray):
        if not self._versions:
            return
        ver = self._versions[-1][1].version
        self._patches[ver].append(
            (t, (x, y, w, h, np.asarray(flat, dtype=np.int8).reshape(h, w)))
        )

    def _version_at(self, t: float) -> Optional[int]:
        if not self._versions:
            return None
        idx: Optional[int] = None
        for (tf, meta, _g) in self._versions:
            if tf <= t:
                idx = meta.version
            else:
                break
        if idx is None:
            idx = self._versions[0][1].version
        return idx

    def snapshot_at(self, t: float) -> Optional[Tuple[_MapMeta, np.ndarray]]:
        if not self._versions:
            return None
        ver = self._version_at(t)
        base_t: Optional[float] = None
        meta: Optional[_MapMeta] = None
        base_grid: Optional[np.ndarray] = None
        for (tf, m, g) in self._versions:
            if m.version == ver:
                base_t, meta, base_grid = tf, m, g
                break
        if meta is None or base_grid is None or base_t is None:
            return None

        if (
            self._cache_version == ver
            and self._cache_grid is not None
            and self._cache_t <= t
        ):
            grid = self._cache_grid.copy(order="C")
            start_t = self._cache_t
        else:
            grid = base_grid.copy(order="C")
            start_t = base_t

        for (tp, patch) in self._patches.get(ver, []):
            if tp <= start_t:
                continue
            if tp > t:
                break
            x, y, w, h, patch_arr = patch
            grid[y : y + h, x : x + w] = patch_arr

        self._cache_version = ver
        self._cache_t = t
        self._cache_grid = grid.copy(order="C")
        return meta, grid


# ----------------- TF timeline helpers -----------------


@dataclass
class _StampedSE2:
    t: float
    x: float
    y: float
    yaw: float


def _se2_compose(a: _StampedSE2, b: _StampedSE2) -> _StampedSE2:
    """
    Return A ∘ B (apply B in A's frame): T(x) = R_a * (R_b * x + t_b) + t_a
    Here we only need translation + yaw composition for 2D.
    """
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    x = a.x + ca * b.x - sa * b.y
    y = a.y + sa * b.x + ca * b.y
    yaw = _wrap_pi(a.yaw + b.yaw)
    return _StampedSE2(t=b.t, x=x, y=y, yaw=yaw)


def _find_latest_before(
    stamps: List[_StampedSE2], t: float, start_idx: int = 0
) -> Tuple[_StampedSE2, int]:
    """
    Return the last transform <= t and the index we reached, so the caller can
    scan forward over time in O(N) total.
    """
    i = max(0, start_idx)
    n = len(stamps)
    if n == 0:
        return _StampedSE2(t=0.0, x=0.0, y=0.0, yaw=0.0), 0
    # advance while next stamp time <= t
    while i + 1 < n and stamps[i + 1].t <= t:
        i += 1
    return stamps[i], i


# ----------------- Scan timeline helpers -----------------


@dataclass
class _Scan:
    t: float
    frame: str
    angle_min: float
    angle_inc: float
    range_min: float
    range_max: float
    ranges: List[float]


class _ScanIndex:
    """
    Simple time-indexed scan list with 'nearest(t)' lookup.
    """

    def __init__(self):
        self._scans: List[_Scan] = []
        self._times: List[float] = []

    def add(self, scan: _Scan):
        self._scans.append(scan)
        self._times.append(scan.t)

    def finalize(self):
        if not self._scans:
            return
        pairs = sorted(zip(self._times, self._scans), key=lambda p: p[0])
        self._times = [p[0] for p in pairs]
        self._scans = [p[1] for p in pairs]

    def nearest(self, t: float, tol: float = 0.05) -> Optional[_Scan]:
        """
        Return scan closest to t if |Δt| <= tol; otherwise None.
        """
        if not self._scans:
            return None
        n = len(self._times)
        # binary search for first time >= t
        lo, hi = 0, n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self._times[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        candidates = [lo]
        if lo > 0:
            candidates.append(lo - 1)
        if lo + 1 < n:
            candidates.append(lo + 1)
        best_idx = candidates[0]
        best_dt = abs(self._times[best_idx] - t)
        for idx in candidates[1:]:
            dt = abs(self._times[idx] - t)
            if dt < best_dt:
                best_dt = dt
                best_idx = idx
        if best_dt <= tol:
            return self._scans[best_idx]
        return None


# ----------------- Cmd_vel timeline helpers -----------------


class _BagCmdIndex:
    """
    Time-indexed cmd_vel (Twist/TwistStamped) list with prefix maxima for basic stats.
    """

    def __init__(self):
        # raw samples before sorting
        self._samples: List[Tuple[float, float, float, float, float, float, float]] = []

        # sorted arrays
        self.times: List[float] = []
        self.vx: List[float] = []
        self.vy: List[float] = []
        self.vz: List[float] = []
        self.wx: List[float] = []
        self.wy: List[float] = []
        self.wz: List[float] = []

        # prefix aggregates
        self.max_vx_forward_prefix: List[float] = []
        self.max_vx_reverse_prefix: List[float] = []
        self.max_wz_abs_prefix: List[float] = []
        self.has_lateral_prefix: List[bool] = []
        self.has_ang_xy_prefix: List[bool] = []

        # overall cmd_vel span in bag time
        self.bag_t0: Optional[float] = None
        self.bag_t1: Optional[float] = None

    def add(
        self,
        t: float,
        vx: float,
        vy: float,
        vz: float,
        wx: float,
        wy: float,
        wz: float,
    ):
        self._samples.append((t, vx, vy, vz, wx, wy, wz))

    def finalize(self):
        if not self._samples:
            return
        # sort by time
        self._samples.sort(key=lambda s: s[0])
        n = len(self._samples)

        self.times = [0.0] * n
        self.vx = [0.0] * n
        self.vy = [0.0] * n
        self.vz = [0.0] * n
        self.wx = [0.0] * n
        self.wy = [0.0] * n
        self.wz = [0.0] * n

        self.max_vx_forward_prefix = [0.0] * n
        self.max_vx_reverse_prefix = [0.0] * n
        self.max_wz_abs_prefix = [0.0] * n
        self.has_lateral_prefix = [False] * n
        self.has_ang_xy_prefix = [False] * n

        max_fwd = 0.0
        max_rev = 0.0  # most negative vx observed (reverse)
        max_wz_abs = 0.0
        has_lat = False
        has_ang_xy = False
        EPS = 1e-6

        for i, (t, vx, vy, vz, wx, wy, wz) in enumerate(self._samples):
            self.times[i] = t
            self.vx[i] = vx
            self.vy[i] = vy
            self.vz[i] = vz
            self.wx[i] = wx
            self.wy[i] = wy
            self.wz[i] = wz

            if vx > max_fwd:
                max_fwd = vx
            if vx < max_rev:
                max_rev = vx
            wz_abs = abs(wz)
            if wz_abs > max_wz_abs:
                max_wz_abs = wz_abs
            if abs(vy) > EPS or abs(vz) > EPS:
                has_lat = True
            if abs(wx) > EPS or abs(wy) > EPS:
                has_ang_xy = True

            self.max_vx_forward_prefix[i] = max_fwd
            self.max_vx_reverse_prefix[i] = max_rev
            self.max_wz_abs_prefix[i] = max_wz_abs
            self.has_lateral_prefix[i] = has_lat
            self.has_ang_xy_prefix[i] = has_ang_xy

        self.bag_t0 = self.times[0]
        self.bag_t1 = self.times[-1]
        # drop the raw list to save memory
        self._samples = []

    def instant_index_at(self, t: float) -> Optional[int]:
        """
        Return index of the last cmd sample at or before time t, or None if none.
        """
        times = self.times
        n = len(times)
        if n == 0:
            return None
        if t < times[0]:
            return None
        if t >= times[-1]:
            return n - 1
        lo, hi = 0, n - 1
        # binary search: last index with times[i] <= t
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return lo


# ----------------- public API -----------------


def list_bag_files():
    os.makedirs(BAG_DIR, exist_ok=True)
    out = []
    for name in sorted(os.listdir(BAG_DIR)):
        if name.endswith(".mcap"):
            p = os.path.join(BAG_DIR, name)
            size = 0
            try:
                size = os.path.getsize(p)
            except OSError:
                pass
            out.append({"name": name, "size": size})
    return out


def replay_bag_in_thread(shared, name: str, speed: float = 1.0):
    """
    Index bag (once) and populate:
      - core.pose_history ([(t,x,y,yaw)...]) in MAP frame if TF is available
      - core.path
      - core._bag_map_index (for /api/v1/map_full_at)
      - core.map_info, core.map_grid, core.map_version (final snapshot)
      - core._bag_scan_index, core._bag_tf_pairs, core._bag_names (for /api/v1/scan_at)
      - core._bag_cmd_index (for /api/v1/cmd_stats_at)
      - core.bag_cmd_* prefix arrays (legacy/compatibility)
    """
    path = os.path.join(BAG_DIR, name)
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    def _worker():
        try:
            print(f"[bag] Opening {path}")
            storage = rosbag2_py.StorageOptions(uri=path, storage_id="mcap")
            converter = rosbag2_py.ConverterOptions("", "")
            reader = rosbag2_py.SequentialReader()
            reader.open(storage, converter)

            topics = _topic_map(reader)
            print(f"[bag] Topics in bag:")
            for k, v in topics.items():
                print(f"   - {k}: {v}")

            ns = _norm_ns(getattr(C, "NAMESPACE", "/a300_0000"))

            # Exact topic preferences
            odom_exact = f"{ns}/platform/odom/filtered"
            map_exact = f"{ns}/map"
            mapup_exact = f"{ns}/map_updates"
            tf_exact = f"{ns}/tf"
            tfstat_exact = f"{ns}/tf_static"
            scan_exact = f"{ns}/sensors/lidar2d_0/scan"
            gc_exact = f"{ns}/global_costmap/costmap"
            lc_exact = f"{ns}/local_costmap/costmap"

            # Msg types
            Odom = get_message("nav_msgs/msg/Odometry")
            OccGrid = get_message("nav_msgs/msg/OccupancyGrid")
            OccUp = get_message("map_msgs/msg/OccupancyGridUpdate")
            TFMsg = get_message("tf2_msgs/msg/TFMessage")
            PoseSt = get_message("geometry_msgs/msg/PoseStamped")
            Laser = get_message("sensor_msgs/msg/LaserScan")
            Twist = get_message("geometry_msgs/msg/Twist")
            TwistStamped = get_message("geometry_msgs/msg/TwistStamped")

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
                # Generic fallback for odom
                for tname, ttype in topics.items():
                    if ttype.endswith("nav_msgs/msg/Odometry"):
                        odom_topic = tname
                        break

            map_topic = pick_exact_or_any(map_exact, "nav_msgs/msg/OccupancyGrid")
            map_up_topic = pick_exact_or_any(
                mapup_exact, "map_msgs/msg/OccupancyGridUpdate"
            )

            # TF topics (accept namespaced and/or global)
            tf_topics: List[str] = []
            for cand in [tf_exact, tfstat_exact, "/tf", "/tf_static"]:
                if cand in topics and topics[cand].endswith("tf2_msgs/msg/TFMessage"):
                    tf_topics.append(cand)
            # Also include any other TFMessage topics present
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

            # Cmd_vel topics (Twist or TwistStamped)
            cmd_topics: List[str] = []
            for tname, ttype in topics.items():
                if (
                    ttype.endswith("geometry_msgs/msg/Twist")
                    or ttype.endswith("geometry_msgs/msg/TwistStamped")
                ):
                    cmd_topics.append(tname)

            # Prefer names that look like cmd_vel in this namespace
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

            # Storage
            poses_odom: List[Tuple[float, float, float, float]] = []  # (t, x_o, y_o, yaw_ob)
            path_pts: List[Tuple[float, float]] = []
            MOVE_EPS = getattr(C, "MOVE_EPS", 0.02)

            map_index = _BagMapIndex()
            gc_index = _BagMapIndex()
            lc_index = _BagMapIndex()
            first_map_meta: Optional[_MapMeta] = None

            # TF timeline candidates
            tf_map_odom: List[_StampedSE2] = []   # map -> odom
            tf_odom_base: List[_StampedSE2] = []  # odom -> base_link/base_footprint

            # Generic TF pairs for scan service
            tf_pairs: Dict[Tuple[str, str], List[_StampedSE2]] = {}

            # Detect frame-name pairs to accept for map->odom and odom->base
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

            # Scan index
            scan_index = _ScanIndex()

            # Cmd_vel index
            cmd_index = _BagCmdIndex()

            # Iterate bag
            while reader.has_next():
                topic, raw, _t_ns = reader.read_next()

                if odom_topic and topic == odom_topic:
                    msg = deserialize_message(raw, Odom)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    px = float(msg.pose.pose.position.x)
                    py = float(msg.pose.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.pose.orientation)
                    poses_odom.append((t, px, py, yaw))
                    if (
                        not path_pts
                        or (px - path_pts[-1][0]) ** 2
                        + (py - path_pts[-1][1]) ** 2
                        >= MOVE_EPS**2
                    ):
                        path_pts.append((px, py))

                elif pose_topic and topic == pose_topic:
                    msg = deserialize_message(raw, PoseSt)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    px = float(msg.pose.position.x)
                    py = float(msg.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.orientation)
                    poses_odom.append((t, px, py, yaw))
                    if (
                        not path_pts
                        or (px - path_pts[-1][0]) ** 2
                        + (py - path_pts[-1][1]) ** 2
                        >= MOVE_EPS**2
                    ):
                        path_pts.append((px, py))

                elif map_topic and topic == map_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = (
                        np.asarray(msg.data, dtype=np.int16)
                        .reshape(h, w)
                        .astype(np.int8, copy=False)
                    )
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    map_index.add_full(
                        t=t,
                        w=w,
                        h=h,
                        res=float(info.resolution),
                        ox=float(info.origin.position.x),
                        oy=float(info.origin.position.y),
                        oyaw=float(oyaw),
                        data=arr,
                    )
                    if first_map_meta is None:
                        first_map_meta = _MapMeta(
                            w,
                            h,
                            float(info.resolution),
                            float(info.origin.position.x),
                            float(info.origin.position.y),
                            float(oyaw),
                            version=1,
                        )

                elif map_up_topic and topic == map_up_topic:
                    msg = deserialize_message(raw, OccUp)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    x0, y0 = int(msg.x), int(msg.y)
                    w, h = int(msg.width), int(msg.height)
                    flat = np.asarray(msg.data, dtype=np.int8)
                    map_index.add_patch(t=t, x=x0, y=y0, w=w, h=h, flat=flat)

                elif gc_topic and topic == gc_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = (
                        np.asarray(msg.data, dtype=np.int16)
                        .reshape(h, w)
                        .astype(np.int8, copy=False)
                    )
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    gc_index.add_full(
                        t=t,
                        w=w,
                        h=h,
                        res=float(info.resolution),
                        ox=float(info.origin.position.x),
                        oy=float(info.origin.position.y),
                        oyaw=float(oyaw),
                        data=arr,
                    )

                elif lc_topic and topic == lc_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = (
                        np.asarray(msg.data, dtype=np.int16)
                        .reshape(h, w)
                        .astype(np.int8, copy=False)
                    )
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    lc_index.add_full(
                        t=t,
                        w=w,
                        h=h,
                        res=float(info.resolution),
                        ox=float(info.origin.position.x),
                        oy=float(info.origin.position.y),
                        oyaw=float(oyaw),
                        data=arr,
                    )

                elif topic in tf_topics:
                    msg = deserialize_message(raw, TFMsg)
                    # Each TFMessage has an array of TransformStamped
                    for ts in msg.transforms:
                        t = float(ts.header.stamp.sec) + float(
                            ts.header.stamp.nanosec
                        ) * 1e-9
                        parent = ts.header.frame_id.strip()
                        child = ts.child_frame_id.strip()
                        x = float(ts.transform.translation.x)
                        y = float(ts.transform.translation.y)
                        yaw = _yaw_from_quat(ts.transform.rotation)
                        se = _StampedSE2(t=t, x=x, y=y, yaw=yaw)

                        # store generic TF pair
                        tf_pairs.setdefault((parent, child), []).append(se)

                        # dedicated map->odom list
                        if (parent in map_candidates) and (child in odom_candidates):
                            tf_map_odom.append(se)

                        # dedicated odom->base list
                        if (parent in odom_candidates) and (child in base_candidates):
                            tf_odom_base.append(se)

                elif scan_topic and topic == scan_topic:
                    msg = deserialize_message(raw, Laser)
                    t = float(msg.header.stamp.sec) + float(
                        msg.header.stamp.nanosec
                    ) * 1e-9
                    frame = (msg.header.frame_id or "").strip()
                    scan = _Scan(
                        t=t,
                        frame=frame,
                        angle_min=float(msg.angle_min),
                        angle_inc=float(msg.angle_increment),
                        range_min=float(msg.range_min),
                        range_max=float(msg.range_max),
                        ranges=[float(r) for r in msg.ranges],
                    )
                    scan_index.add(scan)

                elif topic in cmd_topics:
                    ttype = topics.get(topic, "")
                    if ttype.endswith("geometry_msgs/msg/Twist"):
                        # Plain Twist (no header): use bag timestamp
                        msg = deserialize_message(raw, Twist)
                        t = float(_t_ns) * 1e-9
                        vx = float(getattr(msg.linear, "x", 0.0))
                        vy = float(getattr(msg.linear, "y", 0.0))
                        vz = float(getattr(msg.linear, "z", 0.0))
                        wx = float(getattr(msg.angular, "x", 0.0))
                        wy = float(getattr(msg.angular, "y", 0.0))
                        wz = float(getattr(msg.angular, "z", 0.0))
                        cmd_index.add(t, vx, vy, vz, wx, wy, wz)
                    elif ttype.endswith("geometry_msgs/msg/TwistStamped"):
                        msg = deserialize_message(raw, TwistStamped)
                        t = float(msg.header.stamp.sec) + float(
                            msg.header.stamp.nanosec
                        ) * 1e-9
                        twist = msg.twist
                        vx = float(getattr(twist.linear, "x", 0.0))
                        vy = float(getattr(twist.linear, "y", 0.0))
                        vz = float(getattr(twist.linear, "z", 0.0))
                        wx = float(getattr(twist.angular, "x", 0.0))
                        wy = float(getattr(twist.angular, "y", 0.0))
                        wz = float(getattr(twist.angular, "z", 0.0))
                        cmd_index.add(t, vx, vy, vz, wx, wy, wz)

            # Sort by time
            poses_odom.sort(key=lambda p: p[0])
            tf_map_odom.sort(key=lambda s: s.t)
            tf_odom_base.sort(key=lambda s: s.t)
            scan_index.finalize()
            cmd_index.finalize()

            # ---- Compute poses using TF odom->base (primary), with optional map->odom ----
            poses_final: List[Tuple[float, float, float, float]] = []
            path_pts = []

            if getattr(C, "BAG_USE_TF", True) and tf_odom_base:
                # Use TF odom->base as canonical robot pose source
                if tf_map_odom:
                    # Compose map->odom with odom->base to get map->base
                    poses_map: List[Tuple[float, float, float, float]] = []
                    i_tf = 0
                    for ob in tf_odom_base:
                        mo, i_tf = _find_latest_before(tf_map_odom, ob.t, i_tf)
                        map_of_odom = _StampedSE2(t=mo.t, x=mo.x, y=mo.y, yaw=mo.yaw)
                        odom_of_base = ob
                        mb = _se2_compose(map_of_odom, odom_of_base)
                        poses_map.append((ob.t, mb.x, mb.y, mb.yaw))
                    poses_final = poses_map
                else:
                    # No map->odom: stay in odom frame
                    poses_final = [(ob.t, ob.x, ob.y, ob.yaw) for ob in tf_odom_base]

                # Rebuild path from poses_final
                for (_, x, y, _) in poses_final:
                    if (
                        not path_pts
                        or (x - path_pts[-1][0]) ** 2
                        + (y - path_pts[-1][1]) ** 2
                        >= MOVE_EPS**2
                    ):
                        path_pts.append((x, y))
            else:
                # ---- Fallback: use nav_msgs/Odometry-based pose if TF odom->base is missing ----
                poses_map: List[Tuple[float, float, float, float]] = []
                if getattr(C, "BAG_USE_TF", True) and poses_odom and tf_map_odom:
                    i_tf = 0
                    for (t, xo, yo, yaw_ob) in poses_odom:
                        mo, i_tf = _find_latest_before(tf_map_odom, t, i_tf)
                        # compose T_map_odom ∘ T_odom_base
                        map_of_odom = _StampedSE2(
                            t=mo.t, x=mo.x, y=mo.y, yaw=mo.yaw
                        )
                        odom_of_base = _StampedSE2(
                            t=t, x=xo, y=yo, yaw=yaw_ob
                        )
                        mb = _se2_compose(map_of_odom, odom_of_base)
                        poses_map.append((t, mb.x, mb.y, mb.yaw))
                    # rebuild path in map frame
                    for (_, x, y, _) in poses_map:
                        if (
                            not path_pts
                            or (x - path_pts[-1][0]) ** 2
                            + (y - path_pts[-1][1]) ** 2
                            >= MOVE_EPS**2
                        ):
                            path_pts.append((x, y))
                    poses_final = poses_map
                else:
                    # Anchor odom to map origin if requested
                    poses_final = poses_odom
                    if (
                        getattr(C, "BAG_ANCHOR_ODOM_TO_MAP", True)
                        and poses_odom
                        and map_index._versions
                    ):
                        if first_map_meta is None:
                            first_map_meta = map_index._versions[0][1]
                        mx0 = float(first_map_meta.origin_x)
                        my0 = float(first_map_meta.origin_y)
                        myaw0 = float(first_map_meta.origin_yaw) if getattr(
                            C, "BAG_USE_MAP_YAW", True
                        ) else 0.0

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
                            yawm = _wrap_pi((yaw - yaw0) + myaw0)
                            anchored.append((t, xm, ym, yawm))
                        poses_final = anchored

                    # path in (possibly anchored) frame
                    path_pts = []
                    for (_, x, y, _) in poses_final:
                        if (
                            not path_pts
                            or (x - path_pts[-1][0]) ** 2
                            + (y - path_pts[-1][1]) ** 2
                            >= MOVE_EPS**2
                        ):
                            path_pts.append((x, y))

            # ---- Build cmd_vel prefix arrays from cmd_index ----
            if cmd_index.times:
                bag_cmd_times: List[float] = list(cmd_index.times)
                bag_cmd_vx: List[float] = list(cmd_index.vx)
                bag_cmd_vy: List[float] = list(cmd_index.vy)
                bag_cmd_vz: List[float] = list(cmd_index.vz)
                bag_cmd_wx: List[float] = list(cmd_index.wx)
                bag_cmd_wy: List[float] = list(cmd_index.wy)
                bag_cmd_wz: List[float] = list(cmd_index.wz)

                bag_cmd_prefix_max_vx_fwd: List[float] = list(cmd_index.max_vx_forward_prefix)
                bag_cmd_prefix_max_vx_rev: List[float] = list(cmd_index.max_vx_reverse_prefix)
                bag_cmd_prefix_max_wz_abs: List[float] = list(cmd_index.max_wz_abs_prefix)
                bag_cmd_prefix_has_lateral: List[bool] = list(cmd_index.has_lateral_prefix)
                bag_cmd_prefix_has_ang_xy: List[bool] = list(cmd_index.has_ang_xy_prefix)

                bag_cmd_t0 = cmd_index.bag_t0 if cmd_index.bag_t0 is not None else cmd_index.times[0]
                bag_cmd_t1 = cmd_index.bag_t1 if cmd_index.bag_t1 is not None else cmd_index.times[-1]
            else:
                bag_cmd_times = []
                bag_cmd_vx = []
                bag_cmd_vy = []
                bag_cmd_vz = []
                bag_cmd_wx = []
                bag_cmd_wy = []
                bag_cmd_wz = []

                bag_cmd_prefix_max_vx_fwd = []
                bag_cmd_prefix_max_vx_rev = []
                bag_cmd_prefix_max_wz_abs = []
                bag_cmd_prefix_has_lateral = []
                bag_cmd_prefix_has_ang_xy = []

                bag_cmd_t0 = 0.0
                bag_cmd_t1 = 0.0

            # ---- Commit to shared / core ----
            core = getattr(shared, "core", shared)
            with core.lock:
                # pose history
                max_ph = getattr(C, "POSE_HISTORY_MAX", 100000)
                core.pose_history = deque(poses_final, maxlen=max_ph)

                # path
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

                # map snapshot at end
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

                # lidar + TF info for scan service
                core._bag_scan_index = scan_index if scan_index._scans else None
                core._bag_cmd_index = cmd_index if cmd_index.times else None
                core._bag_tf_pairs = tf_pairs
                core._bag_names = {
                    "ns": ns,
                    "map_candidates": map_candidates,
                    "odom_candidates": odom_candidates,
                    "base_candidates": base_candidates,
                }

                # ---- Commit cmd_vel prefix arrays for /cmd_stats_at ----
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

                core.bag_cmd_t0 = bag_cmd_t0
                core.bag_cmd_t1 = bag_cmd_t1

            print(
                f"[bag] Index complete: poses={len(poses_final)}, path_pts={len(path_pts)}, "
                f"map_versions={len(map_index._versions)}, tf_map_odom={len(tf_map_odom)}, "
                f"tf_odom_base={len(tf_odom_base)}, scans={len(scan_index._scans)}, "
                f"cmd_samples={len(cmd_index.times)}"
            )

        except Exception as e:
            import traceback

            print("BAG INDEX ERROR:", e)
            traceback.print_exc()
            core = getattr(shared, "core", shared)
            with core.lock:
                core._bag_map_index = None
                core._bag_global_costmap_index = None
                core._bag_local_costmap_index = None
                core._bag_scan_index = None
                core._bag_cmd_index = None
                core._bag_tf_pairs = {}
                core._bag_names = {}
                core.bag_cmd_times = []
                core.bag_cmd_vx = []
                core.bag_cmd_vy = []
                core.bag_cmd_vz = []
                core.bag_cmd_wx = []
                core.bag_cmd_wy = []
                core.bag_cmd_wz = []
                core.bag_cmd_prefix_max_vx_fwd = []
                core.bag_cmd_prefix_max_vx_rev = []
                core.bag_cmd_prefix_max_wz_abs = []
                core.bag_cmd_prefix_has_lateral = []
                core.bag_cmd_prefix_has_ang_xy = []
                core.bag_cmd_t0 = 0.0
                core.bag_cmd_t1 = 0.0

    threading.Thread(target=_worker, daemon=True).start()
