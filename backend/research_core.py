# backend/research_core.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Deque, Dict, Any, Tuple, Optional, List
from collections import deque
import numpy as np
import threading

# These lengths match what you had in state.py / config.py
HIST_MAX = 200_000
POSE_MAX = 100_000
PATH_MAX = 10_000
MAP_PATCH_MAX = 2_000


@dataclass
class CmdSample:
    v: float
    w: float
    t: float  # seconds


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float


class ExperimentCapture:
    """
    This is the 'research core' view of the data.
    ROS nodes push into this; FastAPI just reads from it.
    """
    def __init__(self):
        # cmd_vel in/out streams
        self.history_in: Deque[Tuple[float, float, float]] = deque(maxlen=HIST_MAX)
        self.history_out: Deque[Tuple[float, float, float]] = deque(maxlen=HIST_MAX)

        # current live pose (for UI)
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0

        # path for UI
        self.path: Deque[Tuple[float, float]] = deque(maxlen=PATH_MAX)

        # timeline for time slider
        self.pose_history: Deque[Tuple[float, float, float, float]] = deque(maxlen=POSE_MAX)

        # map state
        self.map_grid: Optional[np.ndarray] = None   # int8 [H, W]
        self.map_info: Optional[Dict[str, Any]] = None
        self.map_version: int = 0
        self.map_patch_queue: Deque[Dict[str, Any]] = deque(maxlen=MAP_PATCH_MAX)

        # meta
        self.recording: bool = True
        self.replaying: bool = False
        self.replay_source: str = "out"

        # stats-like fields the UI wants
        self.last_cmd_time: float = 0.0
        self.rx_count: int = 0
        self.last_dt_ms: float = 0.0
        self.last_v: float = 0.0
        self.last_w: float = 0.0

        # concurrency
        self.lock = threading.Lock()

    # ----------------- CMD CAPTURE -----------------
    def record_cmd_in(self, v: float, w: float, t: float):
        with self.lock:
            self.last_cmd_time = t
            self.rx_count += 1
            self.last_v = v
            self.last_w = w
            if self.recording and not self.replaying:
                self.history_in.append((v, w, t))

    def record_cmd_out(self, v: float, w: float, t: float):
        with self.lock:
            if self.recording and not self.replaying:
                self.history_out.append((v, w, t))

    # ----------------- POSE / PATH -----------------
    def update_live_pose(self, t: float, x: float, y: float, yaw: float,
                         move_eps: float = 0.01, sample_every: float = 0.04,
                         last_sample_holder: Dict[str, float] = None):
        """
        Called by ROS (odom/tf/integrator). Keeps a path + timeline.
        We keep last-sample in a dict to avoid making this object ROS-dependent.
        """
        if last_sample_holder is None:
            # fallback if caller doesn't manage last_sample
            last_sample_holder = {"last_t": -1.0}

        with self.lock:
            self.x, self.y, self.yaw = x, y, yaw
            # path densification
            if (not self.path) or ((x - self.path[-1][0]) ** 2 + (y - self.path[-1][1]) ** 2) >= move_eps ** 2:
                self.path.append((x, y))

            # time-sampled pose history
            if (t - last_sample_holder["last_t"]) >= sample_every:
                self.pose_history.append((t, x, y, yaw))
                last_sample_holder["last_t"] = t

    # ----------------- MAP -----------------
    def set_full_map(self, grid: np.ndarray, info: Dict[str, Any], tile_size: int):
        with self.lock:
            self.map_grid = grid
            self.map_version += 1
            info = dict(info)
            info["version"] = self.map_version
            info["tile_size"] = tile_size
            self.map_info = info
            self.map_patch_queue.clear()

    def apply_map_patch(self, patch: Dict[str, Any]):
        # patch: {"x","y","w","h","data":flatlist}
        with self.lock:
            if self.map_grid is None:
                return
            x0 = patch["x"]
            y0 = patch["y"]
            w = patch["w"]
            h = patch["h"]
            data = np.asarray(patch["data"], dtype=np.int8).reshape(h, w)
            self.map_grid[y0:y0 + h, x0:x0 + w] = data
            # queue for WS
            self.map_patch_queue.append(patch)

    # ----------------- UTIL (for app.py) -----------------
    def snapshot_for_ui(self) -> Dict[str, Any]:
        with self.lock:
            path_pts = list(self.path)
            pose = {"x": self.x, "y": self.y, "yaw": self.yaw}
            stats = {
                "rx_count": self.rx_count,
                "last_dt_ms": self.last_dt_ms,
                "hist_in": len(self.history_in),
                "hist_out": len(self.history_out),
                "pose_hist": len(self.pose_history),
            }
            map_meta = self.map_info
        return {"path": path_pts, "pose": pose, "v": self.last_v, "w": self.last_w,
                "stats": stats, "map_meta": map_meta}
