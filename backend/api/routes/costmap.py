# backend/api/routes/costmap.py
"""
Costmap routes.
"""
from fastapi import APIRouter, Query

from ..deps import get_core
from ...services.costmap_service import global_costmap_full_at_service, local_costmap_full_at_service

router = APIRouter(tags=["costmap"])


@router.get("/costmap_global_full_at")
def costmap_global_full_at(t: float = Query(..., description="Absolute bag time")):
    """Get global costmap at a specific bag time."""
    result = global_costmap_full_at_service(get_core(), t)
    if result is None:
        return {"meta": None, "data": []}
    return result


@router.get("/costmap_local_full_at")
def costmap_local_full_at(t: float = Query(..., description="Absolute bag time")):
    """Get local costmap at a specific bag time."""
    result = local_costmap_full_at_service(get_core(), t)
    if result is None:
        return {"meta": None, "data": []}
    return result
