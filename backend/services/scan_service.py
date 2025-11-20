from __future__ import annotations
from typing import List, Tuple, Optional
import math

from ..models import ScanAtResponse

# small struct used in bag_replay; mirror here to avoid import cycles
class _StampedSE2:
    __slots__ = ("t","x","y","yaw")
    def __init__(self, t: float, x: float, y: float, yaw: float):
        self.t = t; self.x = x; self.y = y; self.yaw = yaw

def _wrap_pi(a: float) -> float:
    while a <= -math.pi: a += 2*math.pi
    while a >   math.pi: a -= 2*math.pi
    return a

def _compose(a: _StampedSE2, b: _StampedSE2) -> _StampedSE2:
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    x = a.x + ca * b.x - sa * b.y
    y = a.y + sa * b.x + ca * b.y
    yaw = _wrap_pi(a.yaw + b.yaw)
    return _StampedSE2(t=b.t, x=x, y=y, yaw=yaw)

def _latest_before(lst: List[_StampedSE2], t: float, start_idx: int = 0) -> tuple[Optional[_StampedSE2], int]:
    if not lst: return None, start_idx
    i = max(0, start_idx)
    n = len(lst)
    while i + 1 < n and lst[i + 1].t <= t:
        i += 1
    return lst[i], i

def _pick_pair(tf_pairs: dict, keys: List[Tuple[str,str]]) -> Optional[List[_StampedSE2]]:
    for k in keys:
        if k in tf_pairs:
            return tf_pairs[k]
    return None

def get_scan_at_service(core, t: float, decim: int = 1, max_points: int = 12000, tol: float = 0.05) -> ScanAtResponse:
    """
    Returns point list in MAP frame, nearest scan to |t| within ±tol seconds.
    Uses TF from the bag: map->odom, odom->base, base->scan.
    Falls back to identity for base->scan if not present.
    """
    with core.lock:
        scan_index = getattr(core, "_bag_scan_index", None)
        tf_pairs   = getattr(core, "_bag_tf_pairs", {})
        names      = getattr(core, "_bag_names", {})
        ns = names.get("ns", "")
        map_candidates  = names.get("map_candidates", ["map", f"{ns}/map", "/map"])
        odom_candidates = names.get("odom_candidates", ["odom", f"{ns}/odom", "/odom"])
        base_candidates = names.get("base_candidates", ["base_link", f"{ns}/base_link", "base_footprint",
                                                        f"{ns}/base_footprint", "/base_link", "/base_footprint"])

    if scan_index is None:
        return ScanAtResponse(t=0.0, frame="", count=0, points=[])

    scan = scan_index.nearest(t, tol=tol)
    if scan is None:
        return ScanAtResponse(t=t, frame="", count=0, points=[])

    # Build TF chain at scan.t
    tf_map_odom = _pick_pair(tf_pairs, [(m,o) for m in map_candidates for o in odom_candidates]) or []
    tf_odom_base = _pick_pair(tf_pairs, [(o,b) for o in odom_candidates for b in base_candidates]) or []

    # base->scan frame
    scan_frame = (scan.frame or "").strip() or "base_link"
    tf_base_scan = _pick_pair(tf_pairs, [(b,scan_frame) for b in base_candidates]) or []

    i_mo = i_ob = i_bs = 0
    mo, i_mo = _latest_before(tf_map_odom, scan.t, i_mo)
    ob, i_ob = _latest_before(tf_odom_base, scan.t, i_ob)
    bs, i_bs = _latest_before(tf_base_scan, scan.t, i_bs)

    # Fallbacks
    if ob is None:
        # try to derive from pose_history around t (map frame), then invert mo to get odom
        with core.lock:
            ph = list(core.pose_history)
        # nearest pose
        if ph:
            # binary search
            lo, hi = 0, len(ph) - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if ph[mid][0] < scan.t: lo = mid + 1
                else: hi = mid
            i = lo
            cand = [ph[i]]
            if i > 0: cand.append(ph[i-1])
            tpose, x_m, y_m, yaw_m = min(cand, key=lambda p: abs(p[0] - scan.t))
            # If mo exists, convert map->odom to odom->map and compose to get odom->base ~ inverse(mo)∘(map->base)
            if mo is not None:
                # map->base approx from pose (x_m, y_m, yaw_m)
                mb = _StampedSE2(t=tpose, x=x_m, y=y_m, yaw=yaw_m)
                # invert mo (map->odom) to odom->map
                c, s = math.cos(-mo.yaw), math.sin(-mo.yaw)
                om = _StampedSE2(t=mo.t,
                                 x= - (c*mo.x - s*mo.y),
                                 y= - (s*mo.x + c*mo.y),
                                 yaw = -mo.yaw)
                ob = _compose(om, mb)  # ~ odom->base
    if mo is None:
        # if no map->odom at all, we cannot return map-frame points reliably
        return ScanAtResponse(t=scan.t, frame=scan_frame, count=0, points=[])

    if bs is None:
        # assume base and scan are coincident if we don't have a transform
        bs = _StampedSE2(t=scan.t, x=0.0, y=0.0, yaw=0.0)
    if ob is None:
        # last resort: treat odom->base as zero (unlikely but safe)
        ob = _StampedSE2(t=scan.t, x=0.0, y=0.0, yaw=0.0)

    # Compose map->scan
    mb = _compose(mo, ob)
    ms = _compose(mb, bs)
    cs, ss = math.cos(ms.yaw), math.sin(ms.yaw)

    # Build points with decimation & max cap
    pts: list[float] = []
    N = len(scan.ranges)
    step = max(1, int(decim))
    # rough cap by decimation + explicit max_points
    budget = max_points
    for i in range(0, N, step):
        r = float(scan.ranges[i])
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        ang = scan.angle_min + i * scan.angle_inc
        xs = r * math.cos(ang)
        ys = r * math.sin(ang)
        xm = ms.x + cs * xs - ss * ys
        ym = ms.y + ss * xs + cs * ys
        pts.append(xm); pts.append(ym)
        budget -= 1
        if budget <= 0:
            break

    return ScanAtResponse(t=scan.t, frame=scan_frame, count=len(pts)//2, points=pts)
