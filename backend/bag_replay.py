import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from collections import deque
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import math

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
        self._patches: Dict[int, List[Tuple[float, Tuple[int,int,int,int,np.ndarray]]]] = {}
        self._cache_version: Optional[int] = None
        self._cache_t: float = -1.0
        self._cache_grid: Optional[np.ndarray] = None

    def add_full(self, t: float, w: int, h: int, res: float, ox: float, oy: float, oyaw: float, data: np.ndarray):
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
        self._patches[ver].append((t, (x, y, w, h, np.asarray(flat, dtype=np.int8).reshape(h, w))))

    def _version_at(self, t: float) -> Optional[int]:
        if not self._versions:
            return None
        idx = None
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
        base_t, meta, base_grid = None, None, None
        for (tf, m, g) in self._versions:
            if m.version == ver:
                base_t, meta, base_grid = tf, m, g
                break
        if meta is None:
            return None

        if self._cache_version == ver and self._cache_grid is not None and self._cache_t <= t:
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
            grid[y:y+h, x:x+w] = patch_arr

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

def _find_latest_before(stamps: List[_StampedSE2], t: float, start_idx: int = 0) -> Tuple[_StampedSE2, int]:
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

# ----------------- public API -----------------

def list_bag_files():
    os.makedirs(BAG_DIR, exist_ok=True)
    out = []
    for name in sorted(os.listdir(BAG_DIR)):
        if name.endswith(".mcap"):
            p = os.path.join(BAG_DIR, name)
            size = 0
            try: size = os.path.getsize(p)
            except OSError: pass
            out.append({"name": name, "size": size})
    return out

def replay_bag_in_thread(shared, name: str, speed: float = 1.0):
    """
    Index bag (once) and populate:
      - core.pose_history ([(t,x,y,yaw)...]) in MAP frame if TF is available
      - core.path
      - core._bag_map_index (for /api/v1/map_full_at)
      - core.map_info, core.map_grid, core.map_version (final snapshot)
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
            odom_exact   = f"{ns}/platform/odom/filtered"
            map_exact    = f"{ns}/map"
            mapup_exact  = f"{ns}/map_updates"
            tf_exact     = f"{ns}/tf"
            tfstat_exact = f"{ns}/tf_static"

            # Msg types
            Odom    = get_message("nav_msgs/msg/Odometry")
            OccGrid = get_message("nav_msgs/msg/OccupancyGrid")
            OccUp   = get_message("map_msgs/msg/OccupancyGridUpdate")
            TFMsg   = get_message("tf2_msgs/msg/TFMessage")
            PoseSt  = get_message("geometry_msgs/msg/PoseStamped")  # rarely used fallback

            # Resolve topics
            def pick_exact_or_any(exact_name: str, type_suffix: str) -> Optional[str]:
                if exact_name in topics and topics[exact_name].endswith(type_suffix):
                    return exact_name
                for tname, ttype in topics.items():
                    if ttype.endswith(type_suffix):
                        return tname
                return None

            odom_topic = odom_exact if (odom_exact in topics and topics[odom_exact].endswith("nav_msgs/msg/Odometry")) else None
            if odom_topic is None:
                # No generic fallback for odom unless you want it:
                for tname, ttype in topics.items():
                    if ttype.endswith("nav_msgs/msg/Odometry"):
                        odom_topic = tname; break

            map_topic     = pick_exact_or_any(map_exact, "nav_msgs/msg/OccupancyGrid")
            map_up_topic  = pick_exact_or_any(mapup_exact, "map_msgs/msg/OccupancyGridUpdate")

            # TF topics (accept namespaced and/or global)
            tf_topics: List[str] = []
            for cand in [tf_exact, tfstat_exact, "/tf", "/tf_static"]:
                if cand in topics and topics[cand].endswith("tf2_msgs/msg/TFMessage"):
                    tf_topics.append(cand)
            # Also include any other TFMessage topics present
            for tname, ttype in topics.items():
                if ttype.endswith("tf2_msgs/msg/TFMessage") and tname not in tf_topics:
                    tf_topics.append(tname)

            pose_topic = None
            pose_exact = f"{ns}/pose"
            if odom_topic is None and pose_exact in topics and topics[pose_exact].endswith("geometry_msgs/msg/PoseStamped"):
                pose_topic = pose_exact

            print(f"[bag] Selected topics:")
            print(f"   odom: {odom_topic}")
            print(f"   pose: {pose_topic}")
            print(f"   map:  {map_topic}")
            print(f"   up:   {map_up_topic}")
            print(f"   tf*:  {tf_topics}")

            # Storage
            poses_odom: List[Tuple[float, float, float, float]] = []  # (t, x_o, y_o, yaw_ob)
            path_pts: List[Tuple[float, float]] = []
            MOVE_EPS = getattr(C, "MOVE_EPS", 0.02)

            map_index = _BagMapIndex()
            first_map_meta: Optional[_MapMeta] = None

            # TF timeline candidates for map->odom
            tf_map_odom: List[_StampedSE2] = []

            # Detect frame-name pairs to accept for map->odom
            map_candidates = ["map", f"{ns}/map", "/map"]
            odom_candidates = ["odom", f"{ns}/odom", "/odom"]

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
                    if not path_pts or ((px - path_pts[-1][0])**2 + (py - path_pts[-1][1])**2) >= MOVE_EPS**2:
                        path_pts.append((px, py))

                elif pose_topic and topic == pose_topic:
                    msg = deserialize_message(raw, PoseSt)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    px = float(msg.pose.position.x)
                    py = float(msg.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.orientation)
                    poses_odom.append((t, px, py, yaw))
                    if not path_pts or ((px - path_pts[-1][0])**2 + (py - path_pts[-1][1])**2) >= MOVE_EPS**2:
                        path_pts.append((px, py))

                elif map_topic and topic == map_topic:
                    msg = deserialize_message(raw, OccGrid)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    map_index.add_full(
                        t=t,
                        w=w, h=h, res=float(info.resolution),
                        ox=float(info.origin.position.x),
                        oy=float(info.origin.position.y),
                        oyaw=float(oyaw),
                        data=arr
                    )
                    if first_map_meta is None:
                        first_map_meta = _MapMeta(
                            w, h, float(info.resolution),
                            float(info.origin.position.x),
                            float(info.origin.position.y),
                            float(oyaw), version=1
                        )

                elif map_up_topic and topic == map_up_topic:
                    msg = deserialize_message(raw, OccUp)
                    t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                    x0, y0 = int(msg.x), int(msg.y)
                    w, h = int(msg.width), int(msg.height)
                    flat = np.asarray(msg.data, dtype=np.int8)
                    map_index.add_patch(t=t, x=x0, y=y0, w=w, h=h, flat=flat)

                elif topic in tf_topics:
                    msg = deserialize_message(raw, TFMsg)
                    # Each TFMessage has an array of TransformStamped
                    for ts in msg.transforms:
                        t = float(ts.header.stamp.sec) + float(ts.header.stamp.nanosec) * 1e-9
                        parent = ts.header.frame_id.strip()
                        child  = ts.child_frame_id.strip()
                        # Consider only map->odom pairs (various namespace spellings)
                        if (parent in map_candidates) and (child in odom_candidates):
                            x = float(ts.transform.translation.x)
                            y = float(ts.transform.translation.y)
                            yaw = _yaw_from_quat(ts.transform.rotation)
                            tf_map_odom.append(_StampedSE2(t=t, x=x, y=y, yaw=yaw))

            # Sort by time
            poses_odom.sort(key=lambda p: p[0])
            tf_map_odom.sort(key=lambda s: s.t)

            # ---- Compose into MAP frame if we have TF ----
            poses_map: List[Tuple[float, float, float, float]] = []
            if getattr(C, "BAG_USE_TF", True) and poses_odom and tf_map_odom:
                i_tf = 0
                for (t, xo, yo, yaw_ob) in poses_odom:
                    mo, i_tf = _find_latest_before(tf_map_odom, t, i_tf)
                    # compose T_map_odom ∘ T_odom_base
                    map_of_odom = _StampedSE2(t=mo.t, x=mo.x, y=mo.y, yaw=mo.yaw)
                    odom_of_base = _StampedSE2(t=t, x=xo, y=yo, yaw=yaw_ob)
                    mb = _se2_compose(map_of_odom, odom_of_base)
                    poses_map.append((t, mb.x, mb.y, mb.yaw))
                # rebuild path in map frame
                path_pts = []
                for (_, x, y, _) in poses_map:
                    if not path_pts or ((x - path_pts[-1][0])**2 + (y - path_pts[-1][1])**2) >= MOVE_EPS**2:
                        path_pts.append((x, y))
                poses_final = poses_map
            else:
                # ---- Fallback: anchor odom to map origin (optional yaw) ----
                poses_final = poses_odom
                if getattr(C, "BAG_ANCHOR_ODOM_TO_MAP", True) and poses_odom and map_index._versions:
                    if first_map_meta is None:
                        first_map_meta = map_index._versions[0][1]
                    mx0 = float(first_map_meta.origin_x)
                    my0 = float(first_map_meta.origin_y)
                    myaw0 = float(first_map_meta.origin_yaw) if getattr(C, "BAG_USE_MAP_YAW", True) else 0.0

                    t0, x0, y0, yaw0 = poses_odom[0]
                    dtheta = myaw0 - yaw0
                    c = math.cos(dtheta); s = math.sin(dtheta)

                    anchored: List[Tuple[float,float,float,float]] = []
                    for (t, x, y, yaw) in poses_odom:
                        dx, dy = (x - x0), (y - y0)
                        xr = dx * c - dy * s
                        yr = dx * s + dy * c
                        xm = mx0 + xr
                        ym = my0 + yr
                        yawm = _wrap_pi((yaw - yaw0) + myaw0)
                        anchored.append((t, xm, ym, yawm))
                    poses_final = anchored

                    # path in anchored map frame
                    path_pts = []
                    for (_, x, y, _) in poses_final:
                        if not path_pts or ((x - path_pts[-1][0])**2 + (y - path_pts[-1][1])**2) >= MOVE_EPS**2:
                            path_pts.append((x, y))

            # ---- Commit to shared / core ----
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

            print(f"[bag] Index complete: poses={len(poses_final)}, path_pts={len(path_pts)}, "
                  f"map_versions={len(map_index._versions)}, tf_map_odom={len(tf_map_odom)}")

        except Exception as e:
            import traceback
            print("BAG INDEX ERROR:", e)
            traceback.print_exc()
            core = getattr(shared, "core", shared)
            with core.lock:
                core._bag_map_index = None

    threading.Thread(target=_worker, daemon=True).start()
