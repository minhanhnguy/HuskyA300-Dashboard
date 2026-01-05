# backend/api/routes/pose.py
"""
Pose history routes.
"""
from fastapi import APIRouter

from ..deps import get_core
from ...services.pose_service import get_pose_history_service, get_pose_history_meta_service

router = APIRouter(tags=["pose"])


@router.get("/pose_history")
def pose_history():
    """Get full pose history."""
    return get_pose_history_service(get_core())


@router.get("/pose_history_meta")
def pose_history_meta():
    """Get pose history metadata (count, time range)."""
    return get_pose_history_meta_service(get_core())
