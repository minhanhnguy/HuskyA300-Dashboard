# backend/api/routes/map.py
"""
Map routes.
"""
from fastapi import APIRouter, Query

from ..deps import get_core
from ...services.map_service import map_full_at_service, map_delta_service
from ...core import config as C

router = APIRouter(tags=["map"])


@router.get("/map_meta")
def map_meta():
    """Get map metadata."""
    core = get_core()
    with core.lock:
        info = core.map_info
    return info or {}


@router.get("/map_full")
def map_full():
    """Get full map data (live mode)."""
    core = get_core()
    with core.lock:
        if core.map_grid is None:
            return {"version": 0, "data": []}
        return {
            "version": core.map_version,
            "data": core.map_grid.flatten(order="C").tolist(),
        }


@router.get("/map_full_at")
def map_full_at(t: float = Query(..., description="Absolute bag time (seconds)")):
    """Get full map at a specific bag time."""
    result = map_full_at_service(get_core(), t)
    if result is None:
        return {"meta": None, "data": []}
    return result


@router.get("/map_delta")
def map_delta(
    t0: float = Query(..., description="Start time (seconds, inclusive)"),
    t1: float = Query(..., description="End time (seconds, inclusive)"),
):
    """Get map patches between t0 and t1."""
    result = map_delta_service(get_core(), t0, t1)
    if result is None:
        return {"version": None, "patches": []}
    return result
