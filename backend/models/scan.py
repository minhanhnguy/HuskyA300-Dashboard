# backend/models/scan.py
"""
Pydantic models for lidar scan API responses.
"""
from __future__ import annotations

from typing import List
from pydantic import BaseModel


class ScanAtResponse(BaseModel):
    """
    Lidar scan at (or near) a given bag time, in MAP frame.

    - t:      actual timestamp of the scan we used (seconds, bag time)
    - frame:  original scan frame_id
    - count:  number of (x,y) points
    - points: flattened [x0, y0, x1, y1, ...] in MAP frame (meters)
    """
    t: float
    frame: str
    count: int
    points: List[float]
