# backend/state.py
from dataclasses import dataclass, field
from collections import deque
from threading import Lock
from typing import Deque, Tuple, Optional, Dict, Any

@dataclass
class Stats:
    last_cmd_time: float = 0.0
    rx_count: int = 0
    rx_rate_hz: float = 0.0
    last_dt_ms: float = 0.0
    last_v: float = 0.0
    last_w: float = 0.0

@dataclass
class SharedState:
    # ---- existing pose/path/cmd_vel fields (unchanged) ----
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    path: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=10000))
    max_points: int = 10000
    v: float = 0.0
    w: float = 0.0
    last_update_time: float = 0.0
    stats: Stats = field(default_factory=Stats)
    history_in:  Deque[Tuple[float, float, float]] = field(default_factory=lambda: deque(maxlen=200000))
    history_out: Deque[Tuple[float, float, float]] = field(default_factory=lambda: deque(maxlen=200000))
    replay_source: str = "out"
    pose_history: Deque[Tuple[float, float, float, float]] = field(default_factory=lambda: deque(maxlen=100000))
    recording: bool = True
    replaying: bool = False

    # ---- NEW: map state ----
    map_grid: Optional["np.ndarray"] = None         # np.int8 [H, W]
    map_info: Optional[Dict[str, Any]] = None       # {width,height,resolution,origin:{x,y,yaw},version}
    map_version: int = 0

    # queue of patches to send over WS at 15 Hz
    # each item: {"x":int,"y":int,"w":int,"h":int,"data":list[int]}
    map_patch_queue: Deque[Dict[str, Any]] = field(default_factory=lambda: deque(maxlen=2000))

    # Concurrency
    lock: Lock = field(default_factory=Lock)

    def seed(self, x: float, y: float, yaw: float):
        self.x, self.y, self.yaw = x, y, yaw
        self.path.clear()
        self.path.append((x, y))
    