from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel, ConfigDict, Field


# ---------- Map ----------
class MapOrigin(BaseModel):
    x: float
    y: float
    yaw: float


class MapMeta(BaseModel):
    model_config = ConfigDict(extra="forbid")
    width: int
    height: int
    resolution: float
    origin: MapOrigin
    version: int
    tile_size: int = Field(default=256)


class MapFullResponse(BaseModel):
    # /api/v1/map_full
    version: int
    data: List[int]


class MapFullAtResponse(BaseModel):
    # /api/v1/map_full_at
    meta: MapMeta
    data: List[int]


class MapDeltaPatch(BaseModel):
    x: int
    y: int
    w: int
    h: int
    data: List[int]


class MapDeltaReset(BaseModel):
    meta: MapMeta
    data: List[int]


class MapDeltaResponse(BaseModel):
    # Either a reset, or a set of patches for an existing version
    version: Optional[int] = None
    patches: Optional[List[MapDeltaPatch]] = None
    reset: Optional[MapDeltaReset] = None


# ---------- Pose history ----------
class PoseHistoryItem(BaseModel):
    t: float
    x: float
    y: float
    yaw: float


class PoseHistoryResponse(BaseModel):
    pose_history: List[PoseHistoryItem]


class PoseHistoryMeta(BaseModel):
    count: int
    t0: float
    t1: float


# ---------- Lidar scan ----------
class ScanAtResponse(BaseModel):
    """
    Lidar scan at (or near) a given bag time, in MAP frame.

    - t:      actual timestamp of the scan we used (bag time, seconds)
    - frame:  original scan frame_id (e.g. 'lidar2d_0_laser')
    - count:  number of (x,y) points
    - points: flattened [x0, y0, x1, y1, ...] in MAP frame, meters
    """

    t: float
    frame: str
    count: int
    points: List[float]
