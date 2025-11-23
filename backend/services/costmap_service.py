# backend/services/costmap_service.py
from __future__ import annotations
from typing import Optional, Dict, Any
import numpy as np

from ..models import MapFullAtResponse, MapMeta, MapOrigin
from .. import config as C

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
    meta_model = _meta_to_model(meta)
    data = grid.flatten(order="C").astype(np.int8).tolist()
    return MapFullAtResponse(meta=meta_model, data=data)
