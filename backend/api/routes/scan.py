# backend/api/routes/scan.py
"""
Lidar scan and cmd_vel routes.
"""
from fastapi import APIRouter, Query

from ..deps import get_core
from ...services.scan_service import get_scan_at_service
from ...services.cmd_service import get_cmd_stats_at_service

router = APIRouter(tags=["scan"])


@router.get("/scan_at")
def scan_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
    decim: int = Query(1, ge=1, description="Decimation factor over beam index (1 = all beams)"),
    max_points: int = Query(12000, ge=1, description="Max number of points to return"),
    tol: float = Query(0.05, ge=0.0, description="Time tolerance in seconds for nearest scan"),
):
    """Return lidar scan points in MAP frame nearest to time t (bag mode)."""
    return get_scan_at_service(get_core(), t, decim, max_points, tol)


@router.get("/cmd_stats_at")
def cmd_stats_at(
    t: float = Query(..., description="Absolute bag time (seconds)"),
):
    """Get cmd_vel statistics at a specific bag time."""
    return get_cmd_stats_at_service(get_core(), t)
