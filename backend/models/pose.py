# backend/models/pose.py
"""
Pydantic models for pose history API responses.
"""
from __future__ import annotations

from typing import List
from pydantic import BaseModel


class PoseHistoryItem(BaseModel):
    """A single pose in the history."""
    t: float
    x: float
    y: float
    yaw: float


class PoseHistoryResponse(BaseModel):
    """Response containing the full pose history."""
    pose_history: List[PoseHistoryItem]


class PoseHistoryMeta(BaseModel):
    """Metadata about the pose history."""
    count: int
    t0: float
    t1: float
