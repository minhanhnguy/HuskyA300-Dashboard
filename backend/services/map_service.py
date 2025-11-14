# backend/services/map_service.py
from __future__ import annotations
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

from ..models import (
    MapMeta, MapOrigin, MapFullResponse, MapFullAtResponse,
    MapDeltaPatch, MapDeltaReset, MapDeltaResponse
)
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


def _get_index(core):
    with core.lock:
        return getattr(core, "_bag_map_index", None)


def map_full_at_service(core, t: float) -> Optional[MapFullAtResponse]:
    """
    Exact map at absolute bag time t.
    """
    idx = _get_index(core)
    if idx is None:
        return None

    snap = idx.snapshot_at(t)
    if not snap:
        return None

    meta, grid = snap
    meta_model = _meta_to_model(meta)
    data = grid.flatten(order="C").astype(np.int8).tolist()
    return MapFullAtResponse(meta=meta_model, data=data)


def map_delta_service(core, t0: float, t1: float) -> Optional[MapDeltaResponse]:
    """
    Tile-aligned deltas from t0 -> t1, or a 'reset' payload if the version changed.
    """
    if t1 < t0:
        t0, t1 = t1, t0

    idx = _get_index(core)
    if idx is None:
        return None

    snap1 = idx.snapshot_at(t1)
    if not snap1:
        return MapDeltaResponse(version=None, patches=[])

    meta1, grid1 = snap1
    snap0 = idx.snapshot_at(t0)

    # If no starting map or a version bump -> send a reset (full map at t1)
    if (not snap0) or (snap0[0].version != meta1.version):
        reset = MapDeltaReset(
            meta=_meta_to_model(meta1),
            data=grid1.flatten(order="C").astype(np.int8).tolist(),
        )
        return MapDeltaResponse(reset=reset)

    meta0, grid0 = snap0
    if grid0.shape != grid1.shape:
        reset = MapDeltaReset(
            meta=_meta_to_model(meta1),
            data=grid1.flatten(order="C").astype(np.int8).tolist(),
        )
        return MapDeltaResponse(reset=reset)

    changed = (grid0 != grid1)
    if not np.any(changed):
        return MapDeltaResponse(version=int(meta1.version), patches=[])

    H, W = grid1.shape
    TS = int(getattr(C, "MAP_TILE_SIZE", 256))
    patches: List[MapDeltaPatch] = []

    rows = (H + TS - 1) // TS
    cols = (W + TS - 1) // TS
    for r in range(rows):
        y0 = r * TS
        y1 = min(H, y0 + TS)
        for c in range(cols):
            x0 = c * TS
            x1 = min(W, x0 + TS)
            if np.any(changed[y0:y1, x0:x1]):
                sub = grid1[y0:y1, x0:x1]
                patches.append(
                    MapDeltaPatch(
                        x=int(x0),
                        y=int(y0),
                        w=int(x1 - x0),
                        h=int(y1 - y0),
                        data=sub.flatten(order="C").astype(np.int8).tolist(),
                    )
                )

    return MapDeltaResponse(version=int(meta1.version), patches=patches)
