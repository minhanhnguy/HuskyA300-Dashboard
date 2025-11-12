# backend/bag_replay.py
import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from collections import deque
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
      - core.pose_history ([(t,x,y,yaw)...])
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
            for k,v in topics.items():
                print(f"   - {k}: {v}")

            ns = _norm_ns(getattr(C, "NAMESPACE", "/a300_0000"))

            # Tailored to your setup (prefer filtered)
            odom_candidates_exact = [
                f"{ns}/platform/odom/filtered",
                f"{ns}/platform/odom",
                f"{ns}/robot_localization/odometry/filtered",
                f"{ns}/odom",
            ]
            map_topic_exact   = f"{ns}/map"
            map_updates_exact = f"{ns}/map_updates"

            # Msg types
            Odom     = get_message("nav_msgs/msg/Odometry")
            OccGrid  = get_message("nav_msgs/msg/OccupancyGrid")
            OccUp    = get_message("map_msgs/msg/OccupancyGridUpdate")
            PoseSt   = get_message("geometry_msgs/msg/PoseStamped")

            # Resolve topics (prefer exact)
            def pick_exact_or_any(exact_name: str, type_suffix: str) -> Optional[str]:
                if exact_name in topics and topics[exact_name].endswith(type_suffix):
                    return exact_name
                for tname, ttype in topics.items():  # fallback: any
                    if ttype.endswith(type_suffix):
                        return tname
                return None

            map_topic = pick_exact_or_any(map_topic_exact, "nav_msgs/msg/OccupancyGrid")
            map_up_topic = pick_exact_or_any(map_updates_exact, "map_msgs/msg/OccupancyGridUpdate")

            odom_topic = None
            for cand in odom_candidates_exact:
                if cand in topics and topics[cand].endswith("nav_msgs/msg/Odometry"):
                    odom_topic = cand; break
            if odom_topic is None:
                for tname, ttype in topics.items():
                    if ttype.endswith("nav_msgs/msg/Odometry"):
                        odom_topic = tname; break

            # Last-chance: PoseStamped at /{ns}/pose
            pose_topic = None
            pose_exact = f"{ns}/pose"
            if odom_topic is None and pose_exact in topics and topics[pose_exact].endswith("geometry_msgs/msg/PoseStamped"):
                pose_topic = pose_exact

            print(f"[bag] Selected topics:")
            print(f"   odom: {odom_topic}")
            print(f"   pose: {pose_topic}")
            print(f"   map:  {map_topic}")
            print(f"   up:   {map_up_topic}")

            poses: List[Tuple[float, float, float, float]] = []
            path_pts: List[Tuple[float, float]] = []
            MOVE_EPS = getattr(C, "MOVE_EPS", 0.02)

            map_index = _BagMapIndex()

            # Iterate bag
            while reader.has_next():
                topic, raw, t_ns = reader.read_next()
                t = float(t_ns) * 1e-9

                if odom_topic and topic == odom_topic:
                    msg = deserialize_message(raw, Odom)
                    px = float(msg.pose.pose.position.x)
                    py = float(msg.pose.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.pose.orientation)
                    poses.append((t, px, py, yaw))
                    if not path_pts or ((px - path_pts[-1][0])**2 + (py - path_pts[-1][1])**2) >= MOVE_EPS**2:
                        path_pts.append((px, py))

                elif pose_topic and topic == pose_topic:
                    msg = deserialize_message(raw, PoseSt)
                    px = float(msg.pose.position.x)
                    py = float(msg.pose.position.y)
                    yaw = _yaw_from_quat(msg.pose.orientation)
                    poses.append((t, px, py, yaw))
                    if not path_pts or ((px - path_pts[-1][0])**2 + (py - path_pts[-1][1])**2) >= MOVE_EPS**2:
                        path_pts.append((px, py))

                elif map_topic and topic == map_topic:
                    msg = deserialize_message(raw, OccGrid)
                    info = msg.info
                    w, h = int(info.width), int(info.height)
                    arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w).astype(np.int8, copy=False)
                    oyaw = _yaw_from_quat(info.origin.orientation)
                    map_index.add_full(
                        t=t,
                        w=w, h=h, res=float(info.resolution),
                        ox=float(info.origin.position.x),
                        oy=float(info.origin.position.y),
                        oyaw=oyaw,
                        data=arr
                    )

                elif map_up_topic and topic == map_up_topic:
                    msg = deserialize_message(raw, OccUp)
                    x0, y0 = int(msg.x), int(msg.y)
                    w, h = int(msg.width), int(msg.height)
                    flat = np.asarray(msg.data, dtype=np.int8)
                    map_index.add_patch(t=t, x=x0, y=y0, w=w, h=h, flat=flat)

            # ---- Commit to shared / core (compatible with both shapes) ----
            core = getattr(shared, "core", shared)
            with core.lock:
                # Ensure containers exist
                max_ph = getattr(C, "POSE_HISTORY_MAX", 100000)
                core.pose_history = deque(poses, maxlen=max_ph)

                # path: initialize if missing
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
                end_t = poses[-1][0] if poses else 0.0
                snap = map_index.snapshot_at(end_t)
                if snap:
                    meta, grid = snap
                    core.map_grid = grid
                    # map_version: initialize if missing
                    mv = getattr(core, "map_version", 0)
                    core.map_version = int(mv) + 1
                    core.map_info = {
                        "width": int(meta.width),
                        "height": int(meta.height),
                        "resolution": float(meta.resolution),
                        "origin": {"x": float(meta.origin_x), "y": float(meta.origin_y), "yaw": float(meta.origin_yaw)},
                        "version": int(core.map_version),
                        "tile_size": int(getattr(C, "MAP_TILE_SIZE", 256)),
                    }

                # expose index for /api/v1/map_full_at
                core._bag_map_index = map_index

            print(f"[bag] Index complete: poses={len(poses)}, path_pts={len(path_pts)}, "
                  f"map_versions={len(map_index._versions)}")

        except Exception as e:
            import traceback
            print("BAG INDEX ERROR:", e)
            traceback.print_exc()
            core = getattr(shared, "core", shared)
            with core.lock:
                core._bag_map_index = None

    threading.Thread(target=_worker, daemon=True).start()
