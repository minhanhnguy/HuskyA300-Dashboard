# backend/bag/indexer.py
"""
Index classes for time-travel queries on bag data.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .models import MapMeta, StampedSE2, Scan


def wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def se2_compose(a: StampedSE2, b: StampedSE2) -> StampedSE2:
    """
    Return A ∘ B (apply B in A's frame): T(x) = R_a * (R_b * x + t_b) + t_a
    Here we only need translation + yaw composition for 2D.
    """
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    x = a.x + ca * b.x - sa * b.y
    y = a.y + sa * b.x + ca * b.y
    yaw = wrap_pi(a.yaw + b.yaw)
    return StampedSE2(t=b.t, x=x, y=y, yaw=yaw)


def find_latest_before(
    stamps: List[StampedSE2], t: float, start_idx: int = 0
) -> Tuple[StampedSE2, int]:
    """
    Return the last transform <= t and the index we reached, so the caller can
    scan forward over time in O(N) total.
    """
    i = max(0, start_idx)
    n = len(stamps)
    if n == 0:
        return StampedSE2(t=0.0, x=0.0, y=0.0, yaw=0.0), 0
    # advance while next stamp time <= t
    while i + 1 < n and stamps[i + 1].t <= t:
        i += 1
    return stamps[i], i


class BagMapIndex:
    """
    Keeps full-map versions + patches; reconstructs exact map state at time t.
    """

    def __init__(self):
        self._versions: List[Tuple[float, MapMeta, np.ndarray]] = []
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
        frame_id: str,
        data: np.ndarray,
    ):
        version = len(self._versions) + 1
        meta = MapMeta(w, h, res, ox, oy, oyaw, version, frame_id)
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

    def snapshot_at(self, t: float) -> Optional[Tuple[MapMeta, np.ndarray]]:
        if not self._versions:
            return None
        ver = self._version_at(t)
        base_t: Optional[float] = None
        meta: Optional[MapMeta] = None
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


class ScanIndex:
    """Simple time-indexed scan list with 'nearest(t)' lookup."""

    def __init__(self):
        self._scans: List[Scan] = []
        self._times: List[float] = []

    def add(self, scan: Scan):
        self._scans.append(scan)
        self._times.append(scan.t)

    def finalize(self):
        if not self._scans:
            return
        pairs = sorted(zip(self._times, self._scans), key=lambda p: p[0])
        self._times = [p[0] for p in pairs]
        self._scans = [p[1] for p in pairs]

    def nearest(self, t: float, tol: float = 0.05) -> Optional[Scan]:
        """Return scan closest to t if |Δt| <= tol; otherwise None."""
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


class BagCmdIndex:
    """Time-indexed cmd_vel (Twist/TwistStamped) list with prefix maxima for basic stats."""

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

        self.avg_vx_prefix: List[float] = []
        self.max_vy_abs_prefix: List[float] = []
        self.avg_vy_abs_prefix: List[float] = []

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

        self.avg_vx_prefix = [0.0] * n
        self.max_vy_abs_prefix = [0.0] * n
        self.avg_vy_abs_prefix = [0.0] * n

        max_fwd = 0.0
        max_rev = 0.0
        max_wz_abs = 0.0
        has_lat = False
        has_ang_xy = False

        sum_vx = 0.0
        max_vy_abs = 0.0
        sum_vy_abs = 0.0

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

            sum_vx += vx
            vy_abs_val = abs(vy)
            if vy_abs_val > max_vy_abs:
                max_vy_abs = vy_abs_val
            sum_vy_abs += vy_abs_val

            count = i + 1
            self.max_vx_forward_prefix[i] = max_fwd
            self.max_vx_reverse_prefix[i] = max_rev
            self.max_wz_abs_prefix[i] = max_wz_abs
            self.has_lateral_prefix[i] = has_lat
            self.has_ang_xy_prefix[i] = has_ang_xy

            self.avg_vx_prefix[i] = sum_vx / count
            self.max_vy_abs_prefix[i] = max_vy_abs
            self.avg_vy_abs_prefix[i] = sum_vy_abs / count

        self.bag_t0 = self.times[0]
        self.bag_t1 = self.times[-1]
        self._samples = []

    def instant_index_at(self, t: float) -> Optional[int]:
        """Return index of the last cmd sample at or before time t, or None."""
        times = self.times
        n = len(times)
        if n == 0:
            return None
        if t < times[0]:
            return None
        if t >= times[-1]:
            return n - 1
        lo, hi = 0, n - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return lo


class BagPlanIndex:
    """Time-indexed path (nav_msgs/Path)."""

    def __init__(self):
        self._plans: List[Tuple[float, List[Tuple[float, float]]]] = []

    def add(self, t: float, poses: List[Tuple[float, float]]):
        self._plans.append((t, poses))

    def finalize(self):
        self._plans.sort(key=lambda p: p[0])

    def nearest(self, t: float, tol: float = 0.5) -> Optional[List[Tuple[float, float]]]:
        if not self._plans:
            return None
        lo, hi = 0, len(self._plans) - 1
        best_idx = -1
        best_dt = float("inf")

        while lo <= hi:
            mid = (lo + hi) // 2
            mt = self._plans[mid][0]
            dt = abs(mt - t)
            if dt < best_dt:
                best_dt = dt
                best_idx = mid
            if mt < t:
                lo = mid + 1
            else:
                hi = mid - 1

        if best_idx != -1 and best_dt <= tol:
            return self._plans[best_idx][1]
        return None


class BagGoalIndex:
    """Time-indexed goal pose (geometry_msgs/PoseStamped)."""

    def __init__(self):
        self._goals: List[Tuple[float, float, float, float]] = []  # t, x, y, yaw

    def add(self, t: float, x: float, y: float, yaw: float):
        self._goals.append((t, x, y, yaw))

    def finalize(self):
        self._goals.sort(key=lambda p: p[0])

    def latest_at(self, t: float) -> Optional[Tuple[float, float, float]]:
        """Return the last goal received before or at time t."""
        if not self._goals:
            return None
        lo, hi = 0, len(self._goals) - 1
        idx = -1
        while lo <= hi:
            mid = (lo + hi) // 2
            if self._goals[mid][0] <= t:
                idx = mid
                lo = mid + 1
            else:
                hi = mid - 1

        if idx != -1:
            return self._goals[idx][1:]  # x, y, yaw
        return None
