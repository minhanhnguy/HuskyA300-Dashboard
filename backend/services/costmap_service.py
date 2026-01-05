# backend/services/costmap_service.py
from __future__ import annotations
from typing import Optional, Dict, Any
import numpy as np

from ..models import MapFullAtResponse, MapMeta, MapOrigin
from ..core import config as C
import math
from dataclasses import dataclass

@dataclass
class _StampedSE2:
    t: float
    x: float
    y: float
    yaw: float

def _wrap_pi(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

def _se2_compose(a: _StampedSE2, b: _StampedSE2) -> _StampedSE2:
    """Compose transform A * B."""
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    nx = a.x + (b.x * c - b.y * s)
    ny = a.y + (b.x * s + b.y * c)
    nyaw = _wrap_pi(a.yaw + b.yaw)
    return _StampedSE2(t=b.t, x=nx, y=ny, yaw=nyaw)

def _find_latest_before(sorted_list, t: float, start_idx=0):
    """
    Return (item, index) for the latest item with item.t <= t.
    Assumes sorted_list is sorted by .t
    """
    if not sorted_list:
        return None, 0
    # simple linear scan from start_idx
    best = None
    best_i = start_idx
    for i in range(start_idx, len(sorted_list)):
        item = sorted_list[i]
        if item.t > t:
            break
        best = item
        best_i = i
    if best is None:
        # t is before the first item
        return sorted_list[0], 0
    return best, best_i

def _meta_to_model(meta_like) -> MapMeta:
    """Convert _BagMapIndex meta to a MapMeta pydantic model."""
    return MapMeta(
        width=int(meta_like.width),
        height=int(meta_like.height),
        resolution=float(meta_like.resolution),
        origin=MapOrigin(
            x=float(meta_like.origin_x),
            y=float(meta_like.origin_y),
            yaw=float(meta_like.origin_yaw),
        ),
        version=int(meta_like.version),
        tile_size=int(getattr(C, "MAP_TILE_SIZE", 256)),
    )

def _get_global_index(core):
    with core.lock:
        return getattr(core, "_bag_global_costmap_index", None)

def _get_local_index(core):
    with core.lock:
        return getattr(core, "_bag_local_costmap_index", None)

def global_costmap_full_at_service(core, t: float) -> Optional[MapFullAtResponse]:
    """
    Exact global costmap at absolute bag time t.
    """
    idx = _get_global_index(core)
    if idx is None:
        return None

    snap = idx.snapshot_at(t)
    if not snap:
        return None

    meta, grid = snap
    meta_model = _meta_to_model(meta)
    data = grid.flatten(order="C").astype(np.int8).tolist()
    return MapFullAtResponse(meta=meta_model, data=data)

def local_costmap_full_at_service(core, t: float) -> Optional[MapFullAtResponse]:
    """
    Exact local costmap at absolute bag time t.
    """
    idx = _get_local_index(core)
    if idx is None:
        return None

    snap = idx.snapshot_at(t)
    if not snap:
        return None

    meta, grid = snap
    
    # Check frame_id and transform if necessary
    frame_id = getattr(meta, "frame_id", "map")
    
    # Default origin
    ox, oy, oyaw = meta.origin_x, meta.origin_y, meta.origin_yaw

    if frame_id != "map":
        # Try to find transform map -> frame_id
        tf_pairs = getattr(core, "_bag_tf_pairs", {})
        # We need map -> frame_id. 
        # Usually tf_pairs stores keys like ("map", "odom") -> list of transforms
        # If the costmap is in "odom", we need map->odom.
        
        # Common case: costmap in "odom" or "base_link"
        # We look for ("map", frame_id)
        key = ("map", frame_id)
        transforms = tf_pairs.get(key)
        
        if transforms:
            # Find transform at time t
            tf_at_t, _ = _find_latest_before(transforms, t)
            if tf_at_t:
                # Transform the costmap origin (which is in frame_id) to map frame
                # T_map_frame * T_frame_origin = T_map_origin
                
                # Transform map->frame
                t_map_frame = _StampedSE2(t=tf_at_t.t, x=tf_at_t.x, y=tf_at_t.y, yaw=tf_at_t.yaw)
                
                # Origin in frame
                t_frame_origin = _StampedSE2(t=t, x=ox, y=oy, yaw=oyaw)
                
                # Compose
                t_map_origin = _se2_compose(t_map_frame, t_frame_origin)
                
                ox, oy, oyaw = t_map_origin.x, t_map_origin.y, t_map_origin.yaw

    meta_model = _meta_to_model(meta)
    # Override origin with transformed values
    meta_model.origin.x = ox
    meta_model.origin.y = oy
    meta_model.origin.yaw = oyaw
    
    data = grid.flatten(order="C").astype(np.int8).tolist()
    return MapFullAtResponse(meta=meta_model, data=data)
